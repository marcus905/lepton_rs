use crate::crc::lepton_packet_crc16_spec;

const PACKET_HEADER_BYTES: usize = 4;
const PACKET_DISCARD_MASK: u16 = 0xF000;
const PACKET_NUMBER_MASK: u16 = 0x0FFF;
const SEGMENT_BITS_MASK: u16 = 0x7;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncState {
    Unsynced,
    Seeking,
    Locked,
}

#[derive(Debug, Clone, Copy)]
pub struct RobustCaptureConfig {
    pub enable_crc: bool,
    pub max_resync_attempts: u32,
    pub max_frame_retries: u32,
    pub packet_size_bytes: usize,
    pub lines_per_segment: usize,
    pub segments_per_frame: usize,
    pub max_discard_packets: u32,
    pub timeout_packets: u32,
    pub backoff_packet_reads: u32,
}

impl Default for RobustCaptureConfig {
    fn default() -> Self {
        Self {
            enable_crc: true,
            max_resync_attempts: 20,
            max_frame_retries: 4,
            packet_size_bytes: 164,
            lines_per_segment: 60,
            segments_per_frame: 4,
            max_discard_packets: 600,
            timeout_packets: 3000,
            backoff_packet_reads: 2,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FrameDiagnostics {
    pub discard_count: u32,
    pub crc_error_count: u32,
    pub bad_line_count: u32,
    pub resync_count: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FrameMeta {
    pub valid: bool,
    pub capture_ticks: u64,
    pub discard_packets: u32,
    pub crc_errors: u32,
    pub bad_line_count: u32,
    pub resync_count: u32,
}

#[derive(Debug, Clone)]
pub struct CapturedFrame {
    pub pixels: Vec<u8>,
    pub meta: FrameMeta,
}

#[derive(Debug, Clone, Copy)]
pub struct PacketHeader {
    pub packet_id: u16,
    pub packet_number: u16,
    pub crc: u16,
    pub is_discard: bool,
}

impl PacketHeader {
    pub fn decode_segment_on_packet20(&self) -> Option<u8> {
        if self.packet_number != 20 {
            return None;
        }

        let segment = ((self.packet_id >> 12) & SEGMENT_BITS_MASK) as u8;
        Some(segment)
    }
}

pub fn parse_packet_header(packet: &[u8]) -> Option<PacketHeader> {
    if packet.len() < PACKET_HEADER_BYTES {
        return None;
    }

    let packet_id = u16::from_be_bytes([packet[0], packet[1]]);
    Some(PacketHeader {
        packet_id,
        packet_number: packet_id & PACKET_NUMBER_MASK,
        crc: u16::from_be_bytes([packet[2], packet[3]]),
        is_discard: (packet_id & PACKET_DISCARD_MASK) == PACKET_DISCARD_MASK,
    })
}

pub fn is_discard_packet(packet: &[u8]) -> bool {
    parse_packet_header(packet)
        .map(|h| h.is_discard)
        .unwrap_or(false)
}

pub fn line_number(packet: &[u8]) -> Option<u16> {
    parse_packet_header(packet).map(|h| h.packet_number)
}

pub fn segment_number(packet: &[u8]) -> Option<u8> {
    parse_packet_header(packet).and_then(|h| h.decode_segment_on_packet20())
}

pub fn validate_packet_crc(packet: &[u8]) -> bool {
    if let (Some(header), Some(calc)) = (
        parse_packet_header(packet),
        lepton_packet_crc16_spec(packet),
    ) {
        return header.crc == calc;
    }

    false
}

pub trait PacketSource {
    type Error;
    fn read_packet(&mut self, packet: &mut [u8]) -> Result<(), Self::Error>;
}

pub fn required_frame_buffer_len(cfg: &RobustCaptureConfig) -> usize {
    if cfg.packet_size_bytes < PACKET_HEADER_BYTES {
        return 0;
    }

    (cfg.packet_size_bytes - PACKET_HEADER_BYTES) * cfg.lines_per_segment * cfg.segments_per_frame
}

pub fn capture_frame_from_source<S, F>(
    source: &mut S,
    cfg: &RobustCaptureConfig,
    first_valid_synced: &mut bool,
    sync_state: &mut SyncState,
    diagnostics: &mut FrameDiagnostics,
    mut now_ticks: F,
) -> Result<CapturedFrame, CaptureError<S::Error>>
where
    S: PacketSource,
    F: FnMut() -> u64,
{
    let mut frame = vec![0; required_frame_buffer_len(cfg)];
    let mut packet = vec![0; cfg.packet_size_bytes];
    let meta = capture_frame_into(
        source,
        cfg,
        first_valid_synced,
        sync_state,
        diagnostics,
        &mut frame,
        &mut packet,
        || now_ticks(),
    )?;

    Ok(CapturedFrame {
        pixels: frame,
        meta,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn capture_frame_into<S, F>(
    source: &mut S,
    cfg: &RobustCaptureConfig,
    first_valid_synced: &mut bool,
    sync_state: &mut SyncState,
    diagnostics: &mut FrameDiagnostics,
    frame: &mut [u8],
    packet_buf: &mut [u8],
    mut now_ticks: F,
) -> Result<FrameMeta, CaptureError<S::Error>>
where
    S: PacketSource,
    F: FnMut() -> u64,
{
    if cfg.packet_size_bytes < PACKET_HEADER_BYTES {
        return Err(CaptureError::InvalidPacket);
    }

    if packet_buf.len() < cfg.packet_size_bytes || frame.len() < required_frame_buffer_len(cfg) {
        return Err(CaptureError::InvalidPacket);
    }

    let mut frame_attempts = 0u32;
    let mut resync_attempts = 0u32;
    let mut last_error: Option<CaptureError<S::Error>> = None;

    while frame_attempts <= cfg.max_frame_retries {
        if resync_attempts > cfg.max_resync_attempts {
            *sync_state = SyncState::Unsynced;
            return Err(CaptureError::SyncLost);
        }

        *sync_state = if *first_valid_synced {
            SyncState::Locked
        } else {
            SyncState::Seeking
        };

        let mut meta = FrameMeta {
            capture_ticks: now_ticks(),
            ..FrameMeta::default()
        };

        match read_one_frame(
            source,
            cfg,
            frame,
            packet_buf,
            sync_state,
            diagnostics,
            &mut meta,
        ) {
            Ok(()) => {
                *first_valid_synced = true;
                meta.valid = true;
                return Ok(meta);
            }
            Err(CaptureError::Spi(e)) => return Err(CaptureError::Spi(e)),
            Err(err) => {
                let immediate_locked = *sync_state == SyncState::Locked
                    && matches!(
                        err,
                        CaptureError::CrcMismatch
                            | CaptureError::LineOutOfOrder { .. }
                            | CaptureError::SegmentOutOfOrder { .. }
                    );

                diagnostics.resync_count += 1;
                resync_attempts += 1;
                frame_attempts += 1;
                meta.resync_count += 1;
                *sync_state = SyncState::Unsynced;
                last_error = Some(err);

                if immediate_locked {
                    return Err(last_error.expect("last error set"));
                }

                for _ in 0..cfg.backoff_packet_reads {
                    source
                        .read_packet(&mut packet_buf[..cfg.packet_size_bytes])
                        .map_err(CaptureError::Spi)?;
                }

                if resync_attempts > cfg.max_resync_attempts {
                    return Err(CaptureError::SyncLost);
                }

                if frame_attempts > cfg.max_frame_retries {
                    return Err(last_error.unwrap_or(CaptureError::RetryLimitExceeded));
                }

                continue;
            }
        }
    }

    Err(last_error.unwrap_or(CaptureError::RetryLimitExceeded))
}

fn read_one_frame<S>(
    source: &mut S,
    cfg: &RobustCaptureConfig,
    frame: &mut [u8],
    packet_buf: &mut [u8],
    sync_state: &mut SyncState,
    diagnostics: &mut FrameDiagnostics,
    meta: &mut FrameMeta,
) -> Result<(), CaptureError<S::Error>>
where
    S: PacketSource,
{
    if cfg.packet_size_bytes < PACKET_HEADER_BYTES {
        return Err(CaptureError::InvalidPacket);
    }

    let payload_len = cfg.packet_size_bytes - PACKET_HEADER_BYTES;
    let mut expected_segment = 1usize;
    let mut expected_packet_number = 0usize;
    let mut packets_seen = 0u32;
    let locked = *sync_state == SyncState::Locked;

    while expected_segment <= cfg.segments_per_frame {
        source
            .read_packet(&mut packet_buf[..cfg.packet_size_bytes])
            .map_err(CaptureError::Spi)?;
        packets_seen += 1;

        if packets_seen > cfg.timeout_packets {
            return Err(CaptureError::Timeout);
        }

        let header = parse_packet_header(&packet_buf[..cfg.packet_size_bytes])
            .ok_or(CaptureError::InvalidPacket)?;

        if header.is_discard {
            diagnostics.discard_count += 1;
            meta.discard_packets += 1;
            if meta.discard_packets > cfg.max_discard_packets {
                return Err(CaptureError::DiscardPacketFlood);
            }
            continue;
        }

        if cfg.enable_crc && !validate_packet_crc(&packet_buf[..cfg.packet_size_bytes]) {
            diagnostics.crc_error_count += 1;
            meta.crc_errors += 1;
            if locked {
                return Err(CaptureError::CrcMismatch);
            }
            expected_segment = 1;
            expected_packet_number = 0;
            continue;
        }

        let packet_number = header.packet_number as usize;

        if !locked && expected_segment == 1 && expected_packet_number == 0 && packet_number != 0 {
            continue;
        }

        if packet_number != expected_packet_number {
            if locked {
                diagnostics.bad_line_count += 1;
                meta.bad_line_count += 1;
                return Err(CaptureError::LineOutOfOrder {
                    expected: expected_packet_number as u16,
                    observed: header.packet_number,
                });
            }

            expected_segment = 1;
            expected_packet_number = 0;
            continue;
        }

        if packet_number == 20 {
            let segment = header
                .decode_segment_on_packet20()
                .ok_or(CaptureError::InvalidPacket)?;
            if segment == 0 || segment as usize > cfg.segments_per_frame {
                return Err(CaptureError::InvalidPacket);
            }

            if segment as usize != expected_segment {
                if locked {
                    return Err(CaptureError::SegmentOutOfOrder {
                        expected: expected_segment as u8,
                        observed: segment,
                    });
                }

                expected_segment = 1;
                expected_packet_number = 0;
                continue;
            }
        }

        let frame_line = (expected_segment - 1) * cfg.lines_per_segment + expected_packet_number;
        let dst_start = frame_line * payload_len;
        let dst_end = dst_start + payload_len;
        frame[dst_start..dst_end]
            .copy_from_slice(&packet_buf[PACKET_HEADER_BYTES..cfg.packet_size_bytes]);

        expected_packet_number += 1;
        if expected_packet_number == cfg.lines_per_segment {
            expected_packet_number = 0;
            expected_segment += 1;
        }
    }

    *sync_state = SyncState::Locked;
    Ok(())
}

#[derive(Debug, PartialEq, Eq)]
pub enum CaptureError<SpiError> {
    Spi(SpiError),
    InvalidPacket,
    SyncLost,
    DiscardPacketFlood,
    CrcMismatch,
    SegmentOutOfOrder { expected: u8, observed: u8 },
    LineOutOfOrder { expected: u16, observed: u16 },
    Timeout,
    RetryLimitExceeded,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::lepton_packet_crc16_spec;

    #[derive(Default)]
    struct MockPacketSource {
        packets: Vec<Vec<u8>>,
        idx: usize,
    }

    impl PacketSource for MockPacketSource {
        type Error = ();

        fn read_packet(&mut self, packet: &mut [u8]) -> Result<(), Self::Error> {
            if self.idx >= self.packets.len() {
                return Err(());
            }
            packet.copy_from_slice(&self.packets[self.idx]);
            self.idx += 1;
            Ok(())
        }
    }

    fn mk_packet(
        packet_number: u16,
        segment: u8,
        payload_seed: u8,
        discard_id: Option<u16>,
    ) -> Vec<u8> {
        let mut packet = vec![0u8; 164];
        if let Some(id) = discard_id {
            packet[0..2].copy_from_slice(&id.to_be_bytes());
        } else {
            let id = if packet_number == 20 {
                (((segment as u16) & 0x7) << 12) | (packet_number & 0x0FFF)
            } else {
                packet_number & 0x0FFF
            };
            packet[0..2].copy_from_slice(&id.to_be_bytes());
        }

        for (idx, b) in packet[4..].iter_mut().enumerate() {
            *b = payload_seed.wrapping_add(idx as u8);
        }

        let crc = lepton_packet_crc16_spec(&packet).unwrap();
        packet[2..4].copy_from_slice(&crc.to_be_bytes());
        packet
    }

    fn mk_frame() -> Vec<Vec<u8>> {
        let mut packets = Vec::new();
        for segment in 1..=4 {
            for packet_number in 0..60u16 {
                packets.push(mk_packet(packet_number, segment, (segment * 9) as u8, None));
            }
        }
        packets
    }

    fn run_capture(
        source: &mut MockPacketSource,
        cfg: &RobustCaptureConfig,
    ) -> Result<CapturedFrame, CaptureError<()>> {
        let mut synced = false;
        let mut state = SyncState::Unsynced;
        let mut diag = FrameDiagnostics::default();
        capture_frame_from_source(source, cfg, &mut synced, &mut state, &mut diag, || 1)
    }

    fn run_capture_locked(
        source: &mut MockPacketSource,
        cfg: &RobustCaptureConfig,
    ) -> Result<CapturedFrame, CaptureError<()>> {
        let mut synced = true;
        let mut state = SyncState::Locked;
        let mut diag = FrameDiagnostics::default();
        capture_frame_from_source(source, cfg, &mut synced, &mut state, &mut diag, || 1)
    }

    #[test]
    fn discard_detection_accepts_xfxx_patterns() {
        let p1 = mk_packet(0, 1, 0, Some(0xF001));
        let p2 = mk_packet(0, 1, 0, Some(0xFAAA));
        let p3 = mk_packet(0, 1, 0, Some(0x0AAA));

        assert!(is_discard_packet(&p1));
        assert!(is_discard_packet(&p2));
        assert!(!is_discard_packet(&p3));
    }

    #[test]
    fn segment_decode_only_from_packet_20() {
        let p19 = mk_packet(19, 4, 0, None);
        let p20 = mk_packet(20, 4, 0, None);

        assert_eq!(segment_number(&p19), None);
        assert_eq!(segment_number(&p20), Some(4));
    }

    #[test]
    fn segment_zero_on_packet_20_rejected() {
        let mut packets = mk_frame();
        packets[20] = mk_packet(20, 0, 0, None);
        let mut source = MockPacketSource { packets, idx: 0 };
        let mut cfg = RobustCaptureConfig::default();
        cfg.max_frame_retries = 0;

        let err = run_capture_locked(&mut source, &cfg).unwrap_err();
        assert_eq!(err, CaptureError::InvalidPacket);
    }

    #[test]
    fn crc_validation_detects_payload_corruption() {
        let mut packet = mk_packet(0, 1, 2, None);
        assert!(validate_packet_crc(&packet));
        packet[10] ^= 0xFF;
        assert!(!validate_packet_crc(&packet));
    }

    #[test]
    fn first_garbage_then_valid_frame_recovers() {
        let mut packets = vec![mk_packet(11, 1, 0, None), mk_packet(8, 2, 0, None)];
        packets.extend(mk_frame());

        let mut source = MockPacketSource { packets, idx: 0 };
        let cfg = RobustCaptureConfig::default();

        let frame = run_capture(&mut source, &cfg).unwrap();
        assert!(frame.meta.valid);
        assert_eq!(frame.pixels.len(), 160 * 60 * 4);
    }

    #[test]
    fn wrong_segment_order_rejected_when_locked() {
        let mut packets = mk_frame();
        packets[60 + 20] = mk_packet(20, 3, 0, None);
        let mut source = MockPacketSource { packets, idx: 0 };
        let mut cfg = RobustCaptureConfig::default();
        cfg.max_frame_retries = 0;

        let err = run_capture_locked(&mut source, &cfg).unwrap_err();
        assert_eq!(
            err,
            CaptureError::SegmentOutOfOrder {
                expected: 2,
                observed: 3
            }
        );
    }

    #[test]
    fn line_jump_rejected_when_locked() {
        let mut packets = mk_frame();
        packets[8] = mk_packet(11, 1, 0, None);
        let mut source = MockPacketSource { packets, idx: 0 };
        let mut cfg = RobustCaptureConfig::default();
        cfg.max_frame_retries = 0;

        let err = run_capture_locked(&mut source, &cfg).unwrap_err();
        assert_eq!(
            err,
            CaptureError::LineOutOfOrder {
                expected: 8,
                observed: 11
            }
        );
    }

    #[test]
    fn retries_and_resync_are_bounded() {
        let packets = vec![mk_packet(0, 1, 0, Some(0xF123)); 50];
        let mut source = MockPacketSource { packets, idx: 0 };
        let mut cfg = RobustCaptureConfig::default();
        cfg.max_frame_retries = 2;
        cfg.max_resync_attempts = 1;
        cfg.max_discard_packets = 1;

        let err = run_capture(&mut source, &cfg).unwrap_err();
        assert!(matches!(
            err,
            CaptureError::SyncLost | CaptureError::RetryLimitExceeded
        ));
    }

    #[test]
    fn default_config_matches_telemetry_disabled_frame_geometry() {
        let cfg = RobustCaptureConfig::default();
        assert_eq!(cfg.lines_per_segment, 60);
        assert_eq!(cfg.segments_per_frame, 4);
        assert_eq!(required_frame_buffer_len(&cfg), 160 * 60 * 4);
    }

    #[test]
    fn capture_ticks_uses_supplied_tick_source() {
        let packets = mk_frame();
        let mut source = MockPacketSource { packets, idx: 0 };
        let cfg = RobustCaptureConfig::default();
        let mut synced = false;
        let mut state = SyncState::Unsynced;
        let mut diag = FrameDiagnostics::default();

        let frame = capture_frame_from_source(
            &mut source,
            &cfg,
            &mut synced,
            &mut state,
            &mut diag,
            || 123,
        )
        .unwrap();

        assert_eq!(frame.meta.capture_ticks, 123);
        assert!(frame.meta.valid);
    }
}

use core::fmt;

use crate::lepton_cci::{CciError, LEPTONCCI};
use crate::lepton_status::LepStatus;
use crate::oem::VideoOutputSource;
use crate::vospi::{
    capture_frame_into, required_frame_buffer_len, CaptureError, CapturedFrame, FrameDiagnostics,
    FrameMeta, PacketSource, RobustCaptureConfig, SyncState,
};
use embedded_hal::spi::Operation;
use embedded_hal::{delay::DelayNs, i2c::I2c, spi};

const PACKET_SIZE_BYTES: usize = 164;
const FRAME_PACKETS: usize = 60;

#[derive(Debug, Clone)]
pub struct CameraCheckReport {
    pub tests: Vec<CameraCheckTestResult>,
    pub restored: bool,
}

#[derive(Debug, Clone)]
pub struct CameraCheckTestResult {
    pub name: String,
    pub ok: bool,
    pub details: String,
    pub readback_source: Option<u16>,
}

/// Camera module
pub struct Lepton<I2C, SPI, D> {
    cci: LEPTONCCI<I2C, D>,
    spi: SPI,
    frame: Box<[u8; FRAME_PACKETS * PACKET_SIZE_BYTES]>,
    robust_config: RobustCaptureConfig,
    diagnostics: FrameDiagnostics,
    sync_state: SyncState,
    first_valid_synced: bool,
    packet_buffer: Vec<u8>,
}

impl<I2C, SPI, E1, D> Lepton<I2C, SPI, D>
where
    I2C: I2c<Error = E1>,
    SPI: spi::SpiDevice,
    D: embedded_hal::delay::DelayNs,
    E1: core::fmt::Debug,
{
    fn map_cci_error(err: CciError<E1>) -> LeptonError<E1, SPI::Error> {
        match err {
            CciError::I2c(e) => LeptonError::I2c(e),
            CciError::Timeout => LeptonError::Timeout,
        }
    }

    pub fn new(i2c: I2C, spi: SPI, delay: D) -> Result<Self, E1> {
        let cci = LEPTONCCI::new(i2c, delay)?;
        let robust_config = RobustCaptureConfig::default();
        Ok(Lepton {
            cci,
            spi,
            frame: Box::new([0; FRAME_PACKETS * PACKET_SIZE_BYTES]),
            diagnostics: FrameDiagnostics::default(),
            sync_state: SyncState::Unsynced,
            first_valid_synced: false,
            packet_buffer: vec![0; robust_config.packet_size_bytes],
            robust_config,
        })
    }

    pub fn set_phase_delay(
        &mut self,
        phase_delay: i16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_phase_delay(phase_delay)
            .map_err(Self::map_cci_error)?;
        self.cci.get_status_code().map_err(Self::map_cci_error)
    }

    pub fn get_phase_delay(&mut self) -> Result<(i16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_phase_delay().map_err(Self::map_cci_error)
    }

    pub fn set_gpio_mode(
        &mut self,
        gpio_mode: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_gpio_mode(gpio_mode)
            .map_err(Self::map_cci_error)
    }

    pub fn get_gpio_mode(&mut self) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_gpio_mode().map_err(Self::map_cci_error)
    }

    pub fn set_video_output_format(
        &mut self,
        format: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_oem_video_output_format(format)
            .map_err(Self::map_cci_error)
    }
    pub fn get_video_output_format(
        &mut self,
    ) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci
            .get_oem_video_output_format()
            .map_err(Self::map_cci_error)
    }

    pub fn set_video_output_source(
        &mut self,
        source: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_oem_video_output_source(source)
            .map_err(Self::map_cci_error)
    }

    pub fn get_video_output_source(
        &mut self,
    ) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci
            .get_oem_video_output_source()
            .map_err(Self::map_cci_error)
    }

    pub fn set_video_output_constant(
        &mut self,
        constant: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_oem_video_output_constant(constant)
            .map_err(Self::map_cci_error)
    }

    pub fn get_video_output_constant(
        &mut self,
    ) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci
            .get_oem_video_output_constant()
            .map_err(Self::map_cci_error)
    }

    pub fn get_boot_status(&mut self) -> Result<bool, LeptonError<E1, SPI::Error>> {
        self.cci.get_boot_status().map_err(Self::map_cci_error)
    }

    pub fn get_interface_status(&mut self) -> Result<bool, LeptonError<E1, SPI::Error>> {
        self.cci.get_interface_status().map_err(Self::map_cci_error)
    }

    pub fn set_telemetry_mode(
        &mut self,
        mode: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_telemetry_mode(mode)
            .map_err(Self::map_cci_error)
    }

    pub fn get_telemetry_mode(&mut self) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_telemetry_mode().map_err(Self::map_cci_error)
    }

    pub fn get_agc_enable(&mut self) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_agc_enable().map_err(Self::map_cci_error)
    }

    pub fn set_agc_enable(&mut self, mode: u16) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci.set_agc_enable(mode).map_err(Self::map_cci_error)
    }

    /// Runs an end-to-end camera health check by programming deterministic OEM
    /// video output source patterns and validating one robust VoSPI frame for each.
    ///
    /// This uses the OEM module concepts from Lepton Software IDD Rev 303:
    /// source select (`0x0800:0x2C/0x2D`) and source constant value (`0x0800:0x3C/0x3D`).
    /// The report is non-panicking and includes likely failure causes for CCI,
    /// VoSPI framing/timing, and payload byte-order interpretation.
    pub fn check_camera(&mut self) -> CameraCheckReport {
        let mut report = CameraCheckReport {
            tests: Vec::new(),
            restored: true,
        };

        let original_source = self.get_video_output_source().ok().map(|(value, _)| value);
        let original_constant = self
            .get_video_output_constant()
            .ok()
            .map(|(value, _)| value);

        let tests = [
            (
                "constant_0x1234",
                VideoOutputSource::Constant,
                Some(0x1234u16),
            ),
            (
                "constant_0x2AAA",
                VideoOutputSource::Constant,
                Some(0x2AAAu16),
            ),
            ("ramp_h", VideoOutputSource::RampH, None),
            ("ramp_v", VideoOutputSource::RampV, None),
            ("ramp", VideoOutputSource::Ramp, None),
        ];

        for (name, source, constant) in tests {
            report
                .tests
                .push(self.run_camera_check_test(name, source, constant));
        }

        if let Some(constant) = original_constant {
            if self.set_video_output_constant(constant).is_err() {
                report.restored = false;
            }
        } else {
            report.restored = false;
        }

        if let Some(source) = original_source {
            if self.set_video_output_source(source).is_err() {
                report.restored = false;
            }
        } else {
            report.restored = false;
        }

        report
    }

    fn run_camera_check_test(
        &mut self,
        name: &str,
        source: VideoOutputSource,
        constant: Option<u16>,
    ) -> CameraCheckTestResult {
        if let Some(value) = constant {
            if let Err(err) = self.set_video_output_constant(value) {
                return CameraCheckTestResult {
                    name: name.to_string(),
                    ok: false,
                    details: format!("failed setting constant value via CCI: {}", err),
                    readback_source: None,
                };
            }
        }

        if let Err(err) = self.set_video_output_source(source as u16) {
            return CameraCheckTestResult {
                name: name.to_string(),
                ok: false,
                details: format!("failed setting output source via CCI: {}", err),
                readback_source: None,
            };
        }

        let readback_source = match self.get_video_output_source() {
            Ok((value, _)) => value,
            Err(err) => {
                return CameraCheckTestResult {
                    name: name.to_string(),
                    ok: false,
                    details: format!("failed reading output source readback via CCI: {}", err),
                    readback_source: None,
                }
            }
        };

        if readback_source != source as u16 {
            return CameraCheckTestResult {
                name: name.to_string(),
                ok: false,
                details: "CCI set didn't stick / I2C or busy state".to_string(),
                readback_source: Some(readback_source),
            };
        }

        for _ in 0..2 {
            let _ = self.read_frame_robust();
        }

        let frame =
            match self.read_frame_robust() {
                Ok(frame) => frame,
                Err(_) => return CameraCheckTestResult {
                    name: name.to_string(),
                    ok: false,
                    details:
                        "CCI ok, but robust capture failed: likely SPI framing/timing/clock mode"
                            .to_string(),
                    readback_source: Some(readback_source),
                },
            };

        let payload_bytes_per_packet = self.robust_config.packet_size_bytes.saturating_sub(4);
        let cols = payload_bytes_per_packet / 2;
        let rows = self.robust_config.lines_per_segment * self.robust_config.segments_per_frame;

        let (ok, mut details) = validate_pattern(&frame.pixels, source, cols, rows);
        if !ok {
            let (swapped_ok, _) = validate_pattern_swapped(&frame.pixels, source, cols, rows);
            if swapped_ok {
                details.push_str("; likely endianness/word-order issue");
            }
        }

        CameraCheckTestResult {
            name: name.to_string(),
            ok,
            details,
            readback_source: Some(readback_source),
        }
    }

    /// Returns a u8 vec containing the frame data.
    ///
    /// This method is kept for backward compatibility and reads a 60-packet frame.
    pub fn read_frame(&mut self) -> Result<Vec<u8>, LeptonError<E1, SPI::Error>> {
        let first_packet: [u8; PACKET_SIZE_BYTES];

        loop {
            if let Ok(packet) = self.check_packet() {
                if u16::from_be_bytes([packet[0], packet[1]]) == 0 {
                    first_packet = packet;
                    break;
                }
            }
        }

        let mut frame = vec![0u8; FRAME_PACKETS * PACKET_SIZE_BYTES];

        frame[..PACKET_SIZE_BYTES].copy_from_slice(&first_packet);

        self.spi
            .read(&mut frame[PACKET_SIZE_BYTES..])
            .map_err(LeptonError::Spi)?;

        Ok(frame)
    }

    /// Configure robust VoSPI acquisition behavior for Lepton 3.x/3.5.
    pub fn set_robust_config(&mut self, config: RobustCaptureConfig) {
        self.robust_config = config;
        self.packet_buffer
            .resize(self.robust_config.packet_size_bytes, 0);
    }

    /// Returns the active robust VoSPI acquisition configuration.
    pub fn robust_config(&self) -> RobustCaptureConfig {
        self.robust_config
    }

    /// Returns cumulative diagnostic counters for robust capture attempts.
    pub fn diagnostics(&self) -> FrameDiagnostics {
        self.diagnostics
    }

    /// Acquires one robustly validated frame (Lepton 3.x/3.5) and metadata.
    pub fn read_frame_with_meta(&mut self) -> Result<CapturedFrame, LeptonError<E1, SPI::Error>> {
        self.read_frame_robust()
    }

    /// Allocation-free robust capture into a caller-provided buffer.
    ///
    /// `FrameMeta.capture_ticks` is set to `0` in this convenience wrapper.
    pub fn read_frame_robust_into(
        &mut self,
        out: &mut [u8],
    ) -> Result<FrameMeta, LeptonError<E1, SPI::Error>> {
        self.read_frame_robust_into_with_ticks(out, || 0)
    }

    /// Allocation-free robust capture into a caller-provided buffer with timestamp/tick source.
    ///
    /// Uses the caller-provided monotonic tick source to stamp `FrameMeta.capture_ticks`.
    pub fn read_frame_robust_into_with_ticks<F>(
        &mut self,
        out: &mut [u8],
        mut now_ticks: F,
    ) -> Result<FrameMeta, LeptonError<E1, SPI::Error>>
    where
        F: FnMut() -> u64,
    {
        struct SpiSource<'a, S, D> {
            spi: &'a mut S,
            delay: &'a mut D,
            inter_packet_delay_us: u32,
            inter_packet_delay_discard_us: u32,
        }

        impl<S, D> PacketSource for SpiSource<'_, S, D>
        where
            S: spi::SpiDevice,
            D: DelayNs,
        {
            type Error = S::Error;

            fn read_packet(&mut self, packet: &mut [u8]) -> Result<(), Self::Error> {
                self.spi.transaction(&mut [Operation::Read(packet)])?;

                // Apply inter-packet timing at the packet source boundary so every read path in
                // robust capture (normal, discard/backoff, and resync) gets identical behavior.
                let delay_us = if self.inter_packet_delay_discard_us > 0
                    && crate::vospi::is_discard_packet(packet)
                {
                    self.inter_packet_delay_discard_us
                } else {
                    self.inter_packet_delay_us
                };

                if delay_us > 0 {
                    self.delay.delay_us(delay_us);
                }

                Ok(())
            }
        }

        let required = required_frame_buffer_len(&self.robust_config);
        if out.len() < required {
            return Err(LeptonError::InvalidPacket);
        }

        let mut source = SpiSource {
            spi: &mut self.spi,
            delay: self.cci.delay_mut(),
            inter_packet_delay_us: self.robust_config.inter_packet_delay_us,
            inter_packet_delay_discard_us: self.robust_config.inter_packet_delay_discard_us,
        };

        capture_frame_into(
            &mut source,
            &self.robust_config,
            &mut self.first_valid_synced,
            &mut self.sync_state,
            &mut self.diagnostics,
            out,
            &mut self.packet_buffer,
            || now_ticks(),
        )
        .map_err(LeptonError::from_capture)
    }

    /// Acquires one robustly validated frame (Lepton 3.x/3.5).
    pub fn read_frame_robust(&mut self) -> Result<CapturedFrame, LeptonError<E1, SPI::Error>> {
        let mut frame = vec![0; required_frame_buffer_len(&self.robust_config)];
        let meta = self.read_frame_robust_into(&mut frame)?;

        Ok(CapturedFrame {
            pixels: frame,
            meta,
        })
    }

    /// Helper for shared SPI bus use: caller can hold a lock/mutex around this closure.
    pub fn read_frame_robust_locked<R, F>(
        &mut self,
        lock: F,
    ) -> Result<R, LeptonError<E1, SPI::Error>>
    where
        F: FnOnce(&mut Self) -> Result<R, LeptonError<E1, SPI::Error>>,
    {
        lock(self)
    }

    fn check_packet(&mut self) -> Result<[u8; PACKET_SIZE_BYTES], LeptonError<E1, SPI::Error>> {
        let mut packet = [0_u8; PACKET_SIZE_BYTES];
        self.spi.read(&mut packet).map_err(LeptonError::Spi)?;

        Ok(packet)
    }

    /// Returns a box containing the frame data as an array.
    pub fn get_frame(&mut self) -> &Box<[u8; FRAME_PACKETS * PACKET_SIZE_BYTES]> {
        &self.frame
    }

    /// Sets the frame field on the camera struct to data.
    pub fn set_frame(&mut self, data: &[u8]) -> Result<(), &'static str> {
        if data.len() != FRAME_PACKETS * PACKET_SIZE_BYTES {
            return Err("Data length does not match frame buffer size");
        }
        self.frame.copy_from_slice(data);
        Ok(())
    }
}

fn pixel_at(frame: &[u8], cols: usize, row: usize, col: usize, swapped: bool) -> u16 {
    let byte_index = (row * cols + col) * 2;
    let raw = if swapped {
        u16::from_le_bytes([frame[byte_index], frame[byte_index + 1]])
    } else {
        u16::from_be_bytes([frame[byte_index], frame[byte_index + 1]])
    };
    raw & 0x3FFF
}

fn validate_constant(frame: &[u8], swapped: bool, cols: usize, rows: usize) -> (bool, String) {
    let expected_bytes = rows.saturating_mul(cols).saturating_mul(2);
    if cols == 0 || rows == 0 {
        return (
            false,
            format!("invalid geometry rows={} cols={}", rows, cols),
        );
    }
    if frame.len() < expected_bytes {
        return (
            false,
            format!(
                "payload too small for geometry: got {} bytes, need at least {}",
                frame.len(),
                expected_bytes
            ),
        );
    }

    let mut min_value = u16::MAX;
    let mut max_value = u16::MIN;

    for chunk in frame[..expected_bytes].chunks_exact(2) {
        let value = if swapped {
            u16::from_le_bytes([chunk[0], chunk[1]])
        } else {
            u16::from_be_bytes([chunk[0], chunk[1]])
        } & 0x3FFF;
        min_value = min_value.min(value);
        max_value = max_value.max(value);
    }

    let spread = max_value.saturating_sub(min_value);
    (
        spread <= 2,
        format!(
            "constant spread={} (min={}, max={})",
            spread, min_value, max_value
        ),
    )
}

fn validate_ramp_h(frame: &[u8], swapped: bool, cols: usize, rows: usize) -> (bool, String) {
    let expected_bytes = rows.saturating_mul(cols).saturating_mul(2);
    if cols == 0 || rows == 0 {
        return (
            false,
            format!("invalid geometry rows={} cols={}", rows, cols),
        );
    }
    if frame.len() < expected_bytes {
        return (
            false,
            format!(
                "payload too small for geometry: got {} bytes, need at least {}",
                frame.len(),
                expected_bytes
            ),
        );
    }

    let sample_rows = rows.min(24);
    let mut pass = 0usize;
    for i in 0..sample_rows {
        let row = i * rows / sample_rows;
        let first = pixel_at(frame, cols, row, 0, swapped);
        let last = pixel_at(frame, cols, row, cols - 1, swapped);
        if last > first {
            pass += 1;
        }
    }

    let ratio = if sample_rows == 0 {
        0.0
    } else {
        pass as f32 / sample_rows as f32
    };
    (
        ratio >= 0.8,
        format!(
            "ramp_h rows passing={}/{} ({:.1}%)",
            pass,
            sample_rows,
            ratio * 100.0
        ),
    )
}

fn validate_ramp_v(frame: &[u8], swapped: bool, cols: usize, rows: usize) -> (bool, String) {
    let expected_bytes = rows.saturating_mul(cols).saturating_mul(2);
    if cols == 0 || rows == 0 {
        return (
            false,
            format!("invalid geometry rows={} cols={}", rows, cols),
        );
    }
    if frame.len() < expected_bytes {
        return (
            false,
            format!(
                "payload too small for geometry: got {} bytes, need at least {}",
                frame.len(),
                expected_bytes
            ),
        );
    }

    let sample_cols = cols.min(24);
    let mut pass = 0usize;
    for i in 0..sample_cols {
        let col = i * cols / sample_cols;
        let top = pixel_at(frame, cols, 0, col, swapped);
        let bottom = pixel_at(frame, cols, rows - 1, col, swapped);
        if bottom > top {
            pass += 1;
        }
    }

    let ratio = if sample_cols == 0 {
        0.0
    } else {
        pass as f32 / sample_cols as f32
    };
    (
        ratio >= 0.8,
        format!(
            "ramp_v cols passing={}/{} ({:.1}%)",
            pass,
            sample_cols,
            ratio * 100.0
        ),
    )
}

fn validate_pattern(
    frame: &[u8],
    source: VideoOutputSource,
    cols: usize,
    rows: usize,
) -> (bool, String) {
    match source {
        VideoOutputSource::Constant => validate_constant(frame, false, cols, rows),
        VideoOutputSource::RampH => validate_ramp_h(frame, false, cols, rows),
        VideoOutputSource::RampV => validate_ramp_v(frame, false, cols, rows),
        VideoOutputSource::Ramp => {
            let (h_ok, h_details) = validate_ramp_h(frame, false, cols, rows);
            let (v_ok, v_details) = validate_ramp_v(frame, false, cols, rows);
            (h_ok && v_ok, format!("{}, {}", h_details, v_details))
        }
        _ => (false, "unsupported pattern in check".to_string()),
    }
}

fn validate_pattern_swapped(
    frame: &[u8],
    source: VideoOutputSource,
    cols: usize,
    rows: usize,
) -> (bool, String) {
    match source {
        VideoOutputSource::Constant => validate_constant(frame, true, cols, rows),
        VideoOutputSource::RampH => validate_ramp_h(frame, true, cols, rows),
        VideoOutputSource::RampV => validate_ramp_v(frame, true, cols, rows),
        VideoOutputSource::Ramp => {
            let (h_ok, h_details) = validate_ramp_h(frame, true, cols, rows);
            let (v_ok, v_details) = validate_ramp_v(frame, true, cols, rows);
            (h_ok && v_ok, format!("{}, {}", h_details, v_details))
        }
        _ => (false, "unsupported pattern in check".to_string()),
    }
}

#[derive(Debug)]
pub enum LeptonError<I2C, SPI> {
    Spi(SPI),
    I2c(I2C),
    SyncLost,
    InvalidPacket,
    DiscardPacketFlood,
    CrcMismatch,
    SegmentOutOfOrder,
    LineOutOfOrder,
    Timeout,
    RetryLimitExceeded,
}

impl<I2C, SPI> LeptonError<I2C, SPI> {
    fn from_capture(err: CaptureError<SPI>) -> Self {
        match err {
            CaptureError::Spi(e) => LeptonError::Spi(e),
            CaptureError::InvalidPacket => LeptonError::InvalidPacket,
            CaptureError::SyncLost => LeptonError::SyncLost,
            CaptureError::DiscardPacketFlood => LeptonError::DiscardPacketFlood,
            CaptureError::CrcMismatch => LeptonError::CrcMismatch,
            CaptureError::SegmentOutOfOrder { .. } => LeptonError::SegmentOutOfOrder,
            CaptureError::LineOutOfOrder { .. } => LeptonError::LineOutOfOrder,
            CaptureError::Timeout => LeptonError::Timeout,
            CaptureError::RetryLimitExceeded => LeptonError::RetryLimitExceeded,
        }
    }
}

impl<I2C: fmt::Debug, SPI: fmt::Debug> fmt::Display for LeptonError<I2C, SPI> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LeptonError::Spi(e) => write!(f, "SPI Error: {:?}", e),
            LeptonError::I2c(e) => write!(f, "I2C Error: {:?}", e),
            LeptonError::SyncLost => write!(f, "VoSPI synchronization lost"),
            LeptonError::InvalidPacket => write!(f, "Invalid VoSPI packet"),
            LeptonError::DiscardPacketFlood => write!(f, "Discard packet flood"),
            LeptonError::CrcMismatch => write!(f, "CRC mismatch"),
            LeptonError::SegmentOutOfOrder => write!(f, "Segment out of order"),
            LeptonError::LineOutOfOrder => write!(f, "Line out of order"),
            LeptonError::Timeout => write!(f, "Capture timeout"),
            LeptonError::RetryLimitExceeded => write!(f, "Capture retry limit exceeded"),
        }
    }
}

impl<I2C: fmt::Debug + fmt::Display, SPI: fmt::Debug + fmt::Display> std::error::Error
    for LeptonError<I2C, SPI>
{
}

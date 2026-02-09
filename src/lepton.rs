use core::fmt;

use embedded_hal::{i2c::I2c, spi};

use crate::lepton_cci::LEPTONCCI;
use crate::lepton_status::LepStatus;
use crate::vospi::{
    capture_frame_into, required_frame_buffer_len, CaptureError, CapturedFrame, FrameDiagnostics,
    FrameMeta, PacketSource, RobustCaptureConfig, SyncState,
};

const PACKET_SIZE_BYTES: usize = 164;
const FRAME_PACKETS: usize = 60;

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
            .map_err(LeptonError::I2c)?;
        self.cci.get_status_code().map_err(LeptonError::I2c)
    }

    pub fn get_phase_delay(&mut self) -> Result<(i16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_phase_delay().map_err(LeptonError::I2c)
    }

    pub fn set_gpio_mode(
        &mut self,
        gpio_mode: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci.set_gpio_mode(gpio_mode).map_err(LeptonError::I2c)
    }

    pub fn get_gpio_mode(&mut self) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_gpio_mode().map_err(LeptonError::I2c)
    }

    pub fn set_video_output_format(
        &mut self,
        format: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_oem_video_output_format(format)
            .map_err(LeptonError::I2c)
    }
    pub fn get_video_output_format(
        &mut self,
    ) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci
            .get_oem_video_output_format()
            .map_err(LeptonError::I2c)
    }

    pub fn set_video_output_source(
        &mut self,
        source: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_oem_video_output_source(source)
            .map_err(LeptonError::I2c)
    }

    pub fn get_video_output_source(
        &mut self,
    ) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci
            .get_oem_video_output_source()
            .map_err(LeptonError::I2c)
    }

    pub fn set_video_output_constant(
        &mut self,
        constant: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci
            .set_oem_video_output_constant(constant)
            .map_err(LeptonError::I2c)
    }

    pub fn get_video_output_constant(
        &mut self,
    ) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci
            .get_oem_video_output_constant()
            .map_err(LeptonError::I2c)
    }

    pub fn get_boot_status(&mut self) -> Result<bool, LeptonError<E1, SPI::Error>> {
        self.cci.get_boot_status().map_err(LeptonError::I2c)
    }

    pub fn get_interface_status(&mut self) -> Result<bool, LeptonError<E1, SPI::Error>> {
        self.cci.get_interface_status().map_err(LeptonError::I2c)
    }

    pub fn set_telemetry_mode(
        &mut self,
        mode: u16,
    ) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci.set_telemetry_mode(mode).map_err(LeptonError::I2c)
    }

    pub fn get_telemetry_mode(&mut self) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_telemetry_mode().map_err(LeptonError::I2c)
    }

    pub fn get_agc_enable(&mut self) -> Result<(u16, LepStatus), LeptonError<E1, SPI::Error>> {
        self.cci.get_agc_enable().map_err(LeptonError::I2c)
    }

    pub fn set_agc_enable(&mut self, mode: u16) -> Result<LepStatus, LeptonError<E1, SPI::Error>> {
        self.cci.set_agc_enable(mode).map_err(LeptonError::I2c)
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
    pub fn read_frame_robust_into(
        &mut self,
        out: &mut [u8],
    ) -> Result<FrameMeta, LeptonError<E1, SPI::Error>> {
        self.read_frame_robust_into_with_ticks(out, || 0)
    }

    /// Allocation-free robust capture into a caller-provided buffer with timestamp/tick source.
    pub fn read_frame_robust_into_with_ticks<F>(
        &mut self,
        out: &mut [u8],
        mut now_ticks: F,
    ) -> Result<FrameMeta, LeptonError<E1, SPI::Error>>
    where
        F: FnMut() -> u64,
    {
        struct SpiSource<'a, S>(&'a mut S);

        impl<S> PacketSource for SpiSource<'_, S>
        where
            S: spi::SpiDevice,
        {
            type Error = S::Error;

            fn read_packet(&mut self, packet: &mut [u8]) -> Result<(), Self::Error> {
                self.0.read(packet)
            }
        }

        let required = required_frame_buffer_len(&self.robust_config);
        if out.len() < required {
            return Err(LeptonError::InvalidPacket);
        }

        let mut source = SpiSource(&mut self.spi);
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

use crate::lepton_command::LepCommand;
use crate::lepton_status::LepStatus;
use embedded_hal::i2c::I2c;
use crate::lepton::LeptonError::Timeout;

const CCI_STATUS_INTERFACE_BUSY_BIT: u16 = 1 << 0;
const CCI_STATUS_BOOTED_BIT: u16 = 1 << 2;
const COMMAND_POLL_TIMEOUT_MS: u16 = 1000;

macro_rules! generate_get_set_functions {
    (
        $set_fn_name:ident, $get_fn_name:ident, $param_ty:ty, $set_command:expr, $get_command:expr
    ) => {
        pub fn $set_fn_name(&mut self, value: $param_ty) -> Result<LepStatus, E> {
            self.write_register(Register::CCIDataReg0, &value.to_be_bytes())?;
            let command = $set_command;
            self.write_command(command)?;
            self.poll_status()?;
            self.get_status_code()
        }

        pub fn $get_fn_name(&mut self) -> Result<($param_ty, LepStatus), E> {
            let command = $get_command;
            self.write_command(command)?;
            self.poll_status()?;
            let data = self.read_register(Register::CCIDataReg0)?;
            let status_code = self.get_status_code()?;
            Ok((data as $param_ty, status_code))
        }
    };
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LEPTONCCI<I2C, D> {
    i2c: I2C,
    delay: D,
    address: u8,
}

impl<I2C, D, E> LEPTONCCI<I2C, D>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
    D: embedded_hal::delay::DelayNs,
{
    pub fn new(i2c: I2C, delay: D) -> Result<Self, E> {
        Ok(LEPTONCCI {
            i2c,
            delay,
            address: 0x2a,
        })
    }

    pub fn get_boot_status(&mut self) -> Result<bool, E> {
        let response = self.read_register(Register::CCIStatus)?;
        // Camera has booted when CCI status bit 2 is set.
        Ok((response & CCI_STATUS_BOOTED_BIT) != 0)
    }

    pub fn get_interface_status(&mut self) -> Result<bool, E> {
        let response = self.read_register(Register::CCIStatus)?;
        // CCI status bit 0 is interface busy (1 = busy, 0 = command finished).
        Ok((response & CCI_STATUS_INTERFACE_BUSY_BIT) == 0)
    }

    pub fn get_status_code(&mut self) -> Result<LepStatus, E> {
        let response = self.read_register(Register::CCIStatus)?;
        let status = (response >> 8) as u8;
        Ok(LepStatus::from(status as i8))
    }

    //AGC

    generate_get_set_functions!(
        set_agc_enable,
        get_agc_enable,
        u16,
        LepCommand::set_agc_enable(),
        LepCommand::get_agc_enable()
    );

    //SYS

    generate_get_set_functions!(
        set_telemetry_mode,
        get_telemetry_mode,
        u16,
        LepCommand::set_sys_telemetry_mode(),
        LepCommand::get_sys_telemetry_mode()
    );

    //OEM

    generate_get_set_functions!(
        set_oem_video_output_format,
        get_oem_video_output_format,
        u16,
        LepCommand::set_oem_video_output_format(),
        LepCommand::get_oem_video_output_format()
    );

    generate_get_set_functions!(
        set_oem_video_output_source,
        get_oem_video_output_source,
        u16,
        LepCommand::set_oem_video_output_source(),
        LepCommand::get_oem_video_output_source()
    );

    generate_get_set_functions!(
        set_oem_video_output_constant,
        get_oem_video_output_constant,
        u16,
        LepCommand::set_oem_video_output_source_constant(),
        LepCommand::get_oem_video_output_source_constant()
    );

    generate_get_set_functions!(
        set_gpio_mode,
        get_gpio_mode,
        u16,
        LepCommand::set_oem_gpio_mode(),
        LepCommand::get_oem_gpio_mode()
    );

    generate_get_set_functions!(
        set_phase_delay,
        get_phase_delay,
        i16,
        LepCommand::set_oem_phase_delay(),
        LepCommand::get_oem_phase_delay()
    );

    //RAD

    /// Writes into a register
    #[allow(unused)]
    fn write_register(&mut self, register: Register, payload: &[u8]) -> Result<(), E> {
        // Value that will be written as u8
        let mut write_vec = std::vec::Vec::with_capacity(2 + payload.len());
        let address = register.address().to_be_bytes();
        write_vec.extend_from_slice(&address);
        write_vec.extend_from_slice(payload);
        // i2c write
        self.i2c.write(self.address as u8, &write_vec)
    }

    //Write a command
    fn write_command(&mut self, command: LepCommand) -> Result<(), E> {
        let command_id = command.get_command_id();
        let data_length = command.get_data_length();
        self.write_register(Register::CCIDataLength, &data_length)?;
        self.write_register(Register::CCICommandID, &command_id)
    }

    /// Reads a register using a `write_read` method.
    fn read_register(&mut self, register: Register) -> Result<u16, E> {
        // Buffer for values
        let mut data: [u8; 2] = [0; 2];
        // i2c write_read
        self.i2c.write_read(
            self.address as u8,
            &register.address().to_be_bytes(),
            &mut data,
        )?;
        Ok(u16::from_be_bytes(data))
    }

    fn poll_status(&mut self) -> Result<(), E> {
        for _ in 0..COMMAND_POLL_TIMEOUT_MS {
            let command_finished = self.get_interface_status()?;
            if command_finished {
                return Ok(());
            }

            self.delay.delay_ms(1);
        }

        // FIXME: return a proper error instead of panicking
        panic!("Timeout waiting for command to finish")
    }
}

#[derive(Clone, Copy)]
pub enum Register {
    CCIPower = 0x0000,
    CCIStatus = 0x0002,
    CCICommandID = 0x0004,
    CCIDataLength = 0x0006,
    CCIDataReg0 = 0x0008,
}

impl Register {
    fn address(&self) -> u16 {
        *self as u16
    }
}

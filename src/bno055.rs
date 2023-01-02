pub mod registers;

enum PowerMode {
    Normal,
    LowPower,
    Suspended,
}

#[derive(PartialEq)]
enum OperatingMode {
    ConfigMode,
    AccOnly,
    MagOnly,
    GyroOnly,
    AccMag,
    AccGyro,
    MagGyro,
    AccMagGyro,
    Imu,
    Compass,
    M4G,
    NdofFmcOff,
    Ndof,
}

impl OperatingMode {
    pub fn reg_value(&self) -> u8 {
        match self {
            Self::ConfigMode => 0b0000,
            Self::AccOnly => 0b0001,
            Self::MagOnly => 0b0010,
            Self::GyroOnly => 0b0011,
            Self::AccMag => 0b0100,
            Self::AccGyro => 0b0101,
            Self::MagGyro => 0b0110,
            Self::AccMagGyro => 0b0111,
            Self::Imu => 0b1000,
            Self::Compass => 0b1001,
            Self::M4G => 0b1010,
            Self::NdofFmcOff => 0b1011,
            Self::Ndof => 0b1100,
        }
    }
}

enum Error {
    I2cError(rppal::i2c::Error),
    General(&'static str),
}

impl From<rppal::i2c::Error> for Error {
    fn from(error: rppal::i2c::Error) -> Self {
        Self::I2cError(error)
    }
}

#[allow(unused)]
pub struct Driver<'a> {
    i2c: &'a mut rppal::i2c::I2c,
    i2c_slave_address: u16,
    operating_mode: OperatingMode,
}

impl<'a> Driver<'a> {
    #[allow(unused)]
    pub fn new(
        i2c_slave_address: u16,
        i2c: &'a mut rppal::i2c::I2c,
        operating_mode: OperatingMode,
    ) -> Self {
        Self {
            i2c_slave_address,
            i2c,
            operating_mode: OperatingMode::ConfigMode,
        }
    }

    /// Reads an unsigned 16 bit integer from the given address.
    #[allow(unused)]
    pub fn read_u16(&mut self, address: u8) -> Result<u16, Error> {
        let mut buffer: [u8; 2] = [0; 2];

        self.i2c.block_read(address, &mut buffer)?;

        let value: u16 = u16::from_le_bytes(buffer);

        Ok(value)
    }

    /// Writes an unsigned 16 bit integer to the given address.
    #[allow(unused)]
    pub fn write_u16(&mut self, address: u8, value: u16) -> Result<(), Error> {
        let buffer: [u8; 2] = value.to_le_bytes();

        self.i2c.block_write(address, &buffer);

        Ok(())
    }

    /// Changes the operating mode.
    pub fn change_mode(&mut self, operating_mode: OperatingMode) -> Result<(), Error> {
        // Gets the duration of the mode change, also immediately checks if the mode change
        //  is valid, and if not, it will return an error.
        let duration: std::time::Duration = if self.operating_mode == OperatingMode::ConfigMode
            && operating_mode != OperatingMode::ConfigMode
        {
            std::time::Duration::from_millis(7)
        } else if operating_mode == OperatingMode::ConfigMode
            && self.operating_mode != OperatingMode::ConfigMode
        {
            std::time::Duration::from_millis(19)
        } else {
            return Err(Error::General(
                "Can only set operating mode from and to config.",
            ));
        };

        // Sleeps the required duration for the operation to be performed.
        std::thread::sleep(duration);

        Ok(())
    }
}

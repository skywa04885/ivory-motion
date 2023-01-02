pub const ADDRESS: u8 = 0x3E;

#[derive(PartialEq)]
pub enum OperatingMode {
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

impl super::RegisterValue for OperatingMode {
    fn reg_mask() -> u8 {
        0b00001111
    }

    fn reg_from_u8(mut value: u8) -> Result<Self, super::super::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::ConfigMode),
            0b00000001 => Ok(Self::AccOnly),
            0b00000010 => Ok(Self::MagOnly),
            0b00000011 => Ok(Self::GyroOnly),
            0b00000100 => Ok(Self::AccMag),
            0b00000101 => Ok(Self::AccGyro),
            0b00000110 => Ok(Self::MagGyro),
            0b00000111 => Ok(Self::AccMagGyro),
            0b00001000 => Ok(Self::Imu),
            0b00001001 => Ok(Self::Compass),
            0b00001010 => Ok(Self::M4G),
            0b00001011 => Ok(Self::NdofFmcOff),
            0b00001100 => Ok(Self::Ndof),
            _ => Err(super::super::Error::InvalidRegisterValue)
        }
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::ConfigMode => 0b00000000,
            Self::AccOnly => 0b00000001,
            Self::MagOnly => 0b00000010,
            Self::GyroOnly => 0b00000011,
            Self::AccMag => 0b00000100,
            Self::AccGyro => 0b00000101,
            Self::MagGyro => 0b00000110,
            Self::AccMagGyro => 0b00000111,
            Self::Imu => 0b00001000,
            Self::Compass => 0b00001001,
            Self::M4G => 0b00001010,
            Self::NdofFmcOff => 0b00001011,
            Self::Ndof => 0b00001100,
        }
    }
}

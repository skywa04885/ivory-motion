pub const ADDRESS: u8 = 0x35;

#[derive(PartialEq, Debug)]
pub enum SysCalibStat {
    NotCalibrated,
    BadlyCalibrated,
    DecentlyCalibrated,
    FullyCalibrated,
}

impl super::RegisterValue for SysCalibStat {
    fn reg_mask() -> u8 {
        0b11000000
    }

    fn reg_from_u8(mut value: u8) -> Result<Self, crate::drivers::bno055::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::NotCalibrated),
            0b01000000 => Ok(Self::BadlyCalibrated),
            0b10000000 => Ok(Self::DecentlyCalibrated),
            0b11000000 => Ok(Self::FullyCalibrated),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::NotCalibrated => 0b00000000,
            Self::BadlyCalibrated => 0b01000000,
            Self::DecentlyCalibrated => 0b10000000,
            Self::FullyCalibrated => 0b11000000,
        }
    }
}

#[derive(PartialEq, Debug)]
pub enum GyrCalibStat {
    NotCalibrated,
    BadlyCalibrated,
    DecentlyCalibrated,
    FullyCalibrated,
}

impl super::RegisterValue for GyrCalibStat {
    fn reg_mask() -> u8 {
        0b00110000
    }

    fn reg_from_u8(mut value: u8) -> Result<Self, crate::drivers::bno055::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::NotCalibrated),
            0b00010000 => Ok(Self::BadlyCalibrated),
            0b00100000 => Ok(Self::DecentlyCalibrated),
            0b00110000 => Ok(Self::FullyCalibrated),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::NotCalibrated => 0b00000000,
            Self::BadlyCalibrated => 0b00010000,
            Self::DecentlyCalibrated => 0b00100000,
            Self::FullyCalibrated => 0b00110000,
        }
    }
}

#[derive(PartialEq, Debug)]
pub enum AccCalibStat {
    NotCalibrated,
    BadlyCalibrated,
    DecentlyCalibrated,
    FullyCalibrated,
}

impl super::RegisterValue for AccCalibStat {
    fn reg_mask() -> u8 {
        0b00001100
    }

    fn reg_from_u8(mut value: u8) -> Result<Self, crate::drivers::bno055::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::NotCalibrated),
            0b00000100 => Ok(Self::BadlyCalibrated),
            0b00001000 => Ok(Self::DecentlyCalibrated),
            0b00001100 => Ok(Self::FullyCalibrated),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::NotCalibrated => 0b00000000,
            Self::BadlyCalibrated => 0b00000100,
            Self::DecentlyCalibrated => 0b00001000,
            Self::FullyCalibrated => 0b00001100,
        }
    }
}

#[derive(PartialEq, Debug)]
pub enum MagCalibStat {
    NotCalibrated,
    BadlyCalibrated,
    DecentlyCalibrated,
    FullyCalibrated,
}

impl super::RegisterValue for MagCalibStat {
    fn reg_mask() -> u8 {
        0b00000011
    }

    fn reg_from_u8(mut value: u8) -> Result<Self, crate::drivers::bno055::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::NotCalibrated),
            0b00000001 => Ok(Self::BadlyCalibrated),
            0b00000010 => Ok(Self::DecentlyCalibrated),
            0b00000011 => Ok(Self::FullyCalibrated),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::NotCalibrated => 0b00000000,
            Self::BadlyCalibrated => 0b00000001,
            Self::DecentlyCalibrated => 0b00000010,
            Self::FullyCalibrated => 0b00000011,
        }
    }
}

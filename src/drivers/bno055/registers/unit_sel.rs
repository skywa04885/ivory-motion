pub const ADDRESS: u8 = 0x3B;

pub enum Acceleration {
    Mpss,
    MilliG,
}

impl super::RegisterValue for Acceleration {
    fn reg_from_u8(mut value: u8) -> Result<Self, super::super::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::Mpss),
            0b00000001 => Ok(Self::MilliG),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_mask() -> u8 {
        0b00000001
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::Mpss => 0b00000000,
            Self::MilliG => 0b00000001,
        }
    }
}

pub enum AngularRate {
    Dps,
    Rps,
}

impl super::RegisterValue for AngularRate {
    fn reg_from_u8(mut value: u8) -> Result<Self, super::super::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::Dps),
            0b00000010 => Ok(Self::Rps),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_mask() -> u8 {
        0b00000010
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::Dps => 0b00000000,
            Self::Rps => 0b00000010,
        }
    }
}

pub enum EulerAngles {
    Degrees,
    Radians,
}

impl super::RegisterValue for EulerAngles {
    fn reg_from_u8(mut value: u8) -> Result<Self, super::super::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::Degrees),
            0b00000100 => Ok(Self::Radians),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_mask() -> u8 {
        0b00000100
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::Degrees => 0b00000000,
            Self::Radians => 0b00000100,
        }
    }
}

pub enum Temperature {
    Celsius,
    Fahrenheit,
}

impl super::RegisterValue for Temperature {
    fn reg_from_u8(mut value: u8) -> Result<Self, super::super::Error> {
        value &= !Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::Celsius),
            0b00010000 => Ok(Self::Fahrenheit),
            _ => Err(super::super::Error::InvalidRegisterValue),
        }
    }

    fn reg_mask() -> u8 {
        0b00010000
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::Celsius => 0b00000000,
            Self::Fahrenheit => 0b00010000,
        }
    }
}

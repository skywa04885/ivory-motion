pub const ADDRESS: u8 = 0x3E;

pub enum PowerMode {
    Normal,
    LowPower,
    Suspended,
}

impl super::RegisterValue for PowerMode {
    fn reg_from_u8(mut value: u8) -> Result<Self, super::super::Error> {
        value &= Self::reg_mask();

        match value {
            0b00000000 => Ok(Self::Normal),
            0b00000001 => Ok(Self::LowPower),
            0b00000010 => Ok(Self::Suspended),
            _ => Err(super::super::Error::InvalidRegisterValue)
        }
    }

    fn reg_mask() -> u8 {
        0b00000011
    }

    fn reg_value(&self) -> u8 {
        match self {
            Self::Normal => 0b00000000,
            Self::LowPower => 0b00000001,
            Self::Suspended => 0b00000010
        }
    }
}
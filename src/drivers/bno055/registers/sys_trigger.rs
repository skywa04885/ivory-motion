pub const ADDRESS: u8 = 0x3F;

pub enum RstSys {
  DoNotReset,
  Reset
}

impl super::RegisterValue for RstSys {
  fn reg_mask() -> u8 {
      0b00100000
  }

  fn reg_from_u8(mut value: u8) -> Result<Self, crate::drivers::bno055::Error> {
      value &= Self::reg_mask();

      match value {
        0b00000000 => Ok(Self::DoNotReset),
        0b00100000 => Ok(Self::Reset),
        _ => Err(super::super::Error::InvalidRegisterValue)
      }
  }

  fn reg_value(&self) -> u8 {
      match self {
        Self::DoNotReset => 0b00000000,
        Self::Reset => 0b00100000,
      }
  }
}
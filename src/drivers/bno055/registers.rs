pub mod unit_sel;
pub mod opr_mode;
pub mod calib_stat;
pub mod power_mode;

pub trait RegisterValue: Sized {
    fn reg_mask() -> u8;
    fn reg_from_u8(value: u8) -> Result<Self, super::Error>;
    fn reg_value(&self) -> u8;
}

pub const ACC_RADIUS_LSB: u8 = 0x67;
pub const GYR_Z_OFFSET_LSB: u8 = 0x65;
pub const GYR_Y_OFFSET_LSB: u8 = 0x63;
pub const GYR_X_OFFSET_LSB: u8 = 0x61;
pub const ACC_OFFSET_Z_LSB: u8 = 0x59;
pub const ACC_OFFSET_Y_LSB: u8 = 0x57;
pub const ACC_OFFSET_X_LSB: u8 = 0x55;
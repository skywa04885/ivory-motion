pub mod unit_sel;
pub mod opr_mode;
pub mod calib_stat;
pub mod power_mode;
pub mod sys_trigger;

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
pub const EUL_PITCH_LSB: u8 = 0x1E;
pub const EUL_ROLL_LSB: u8 = 0x1C;
pub const EUL_YAW_LSB: u8 = 0x1A;
pub const LIA_DATA_X_LSB: u8 = 0x28;
pub const LIA_DATA_Y_LSB: u8 = 0x2A;
pub const LIA_DATA_Z_LSB: u8 = 0x2C;
pub const GRV_DATA_X_LSB: u8 = 0x2E;
pub const GRV_DATA_Y_LSB: u8 = 0x30;
pub const GRV_DATA_Z_LSB: u8 = 0x32;
pub const TEMP: u8 = 0x34;
pub const QUA_DATA_W_LSB: u8 = 0x20;
pub const QUA_DATA_X_LSB: u8 = 0x22;
pub const QUA_DATA_Y_LSB: u8 = 0x24;
pub const QUA_DATA_Z_LSB: u8 = 0x26;
pub const CHIP_ID: u8 = 0x00;
pub const ACC_ID: u8 = 0x01;
pub const GYR_ID: u8 = 0x03;
pub const SW_REV_ID_LSB: u8 = 0x04;
pub const PAGE_ID: u8 = 0x07;
pub const ACC_DATA_Z_LSB: u8 = 0x0C;
pub const ACC_DATA_Y_LSB: u8 = 0x0A;
pub const ACC_DATA_X_LSB: u8 = 0x08;

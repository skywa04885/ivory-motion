use nalgebra::{Vector, Vector3};

pub mod registers;

#[derive(Debug)]
pub enum Error {
    I2cError(rppal::i2c::Error),
    InvalidRegisterValue,
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
    euler_angles_unit: Option<registers::unit_sel::EulerAngles>,
    acceleration_unit: Option<registers::unit_sel::Acceleration>,
    temperature_unit: Option<registers::unit_sel::Temperature>,
}

impl<'a> Driver<'a> {
    #[allow(unused)]
    pub fn new(i2c_slave_address: u16, i2c: &'a mut rppal::i2c::I2c) -> Self {
        Self {
            i2c_slave_address,
            i2c,
            euler_angles_unit: None,
            acceleration_unit: None,
            temperature_unit: None,
        }
    }

    #[allow(unused)]
    pub fn set_register_value<T: registers::RegisterValue>(
        &mut self,
        address: u8,
        new_value: T,
    ) -> Result<(), Error> {
        let mut current_value: u8 = self.read_u8(address)?;

        current_value &= !T::reg_mask();
        current_value |= new_value.reg_value();

        self.write_u8(address, current_value)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn get_register_value<T: registers::RegisterValue>(
        &mut self,
        address: u8,
    ) -> Result<T, Error> {
        let current_value: u8 = self.read_u8(address)?;

        T::reg_from_u8(current_value)
    }

    #[allow(unused)]
    pub fn read_u8(&mut self, address: u8) -> Result<u8, Error> {
        let mut buffer: [u8; 1] = [0; 1];

        self.i2c.block_read(address, &mut buffer)?;

        let value: u8 = u8::from_le_bytes(buffer);

        Ok(value)
    }

    #[allow(unused)]
    pub fn write_u8(&mut self, address: u8, value: u8) -> Result<(), Error> {
        let buffer: [u8; 1] = value.to_le_bytes();

        self.i2c.block_write(address, &buffer)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn read_u16(&mut self, address: u8) -> Result<u16, Error> {
        let mut buffer: [u8; 2] = [0; 2];

        self.i2c.block_read(address, &mut buffer)?;

        let value: u16 = u16::from_le_bytes(buffer);

        Ok(value)
    }

    #[allow(unused)]
    pub fn write_u16(&mut self, address: u8, value: u16) -> Result<(), Error> {
        let buffer: [u8; 2] = value.to_le_bytes();

        self.i2c.block_write(address, &buffer)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn read_i16(&mut self, address: u8) -> Result<i16, Error> {
        let mut buffer: [u8; 2] = [0; 2];

        self.i2c.block_read(address, &mut buffer)?;

        let value: i16 = i16::from_le_bytes(buffer);

        Ok(value)
    }

    #[allow(unused)]
    pub fn write_i16(&mut self, address: u8, value: i16) -> Result<(), Error> {
        let buffer: [u8; 2] = value.to_le_bytes();

        self.i2c.block_write(address, &buffer)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn set_acceleration_unit(
        &mut self,
        acceleration_unit: registers::unit_sel::Acceleration,
    ) -> Result<(), Error> {
        self.acceleration_unit = Some(acceleration_unit.clone());
        self.set_register_value(registers::unit_sel::ADDRESS, acceleration_unit)
    }

    #[allow(unused)]
    pub fn get_acceleration_unit(&mut self) -> Result<registers::unit_sel::Acceleration, Error> {
        if self.acceleration_unit.is_some() {
            return Ok(self.acceleration_unit.as_ref().unwrap().clone());
        }

        self.acceleration_unit = Some(self.get_register_value(registers::unit_sel::ADDRESS)?);

        Ok(self.acceleration_unit.as_ref().unwrap().clone())
    }

    #[allow(unused)]
    pub fn set_angular_rate_unit(
        &mut self,
        angular_rate_unit: registers::unit_sel::AngularRate,
    ) -> Result<(), Error> {
        self.set_register_value(registers::unit_sel::ADDRESS, angular_rate_unit)
    }

    #[allow(unused)]
    pub fn get_angular_rate_unit(&mut self) -> Result<registers::unit_sel::AngularRate, Error> {
        self.get_register_value(registers::unit_sel::ADDRESS)
    }

    #[allow(unused)]
    pub fn set_euler_angles_unit(
        &mut self,
        euler_angles_unit: registers::unit_sel::EulerAngles,
    ) -> Result<(), Error> {
        self.euler_angles_unit = Some(euler_angles_unit.clone());
        self.set_register_value(registers::unit_sel::ADDRESS, euler_angles_unit)
    }

    #[allow(unused)]
    pub fn get_euler_angles_unit(&mut self) -> Result<registers::unit_sel::EulerAngles, Error> {
        if self.euler_angles_unit.is_some() {
            return Ok(self.euler_angles_unit.as_ref().unwrap().clone());
        }

        self.euler_angles_unit = Some(self.get_register_value(registers::unit_sel::ADDRESS)?);

        Ok(self.euler_angles_unit.as_ref().unwrap().clone())
    }

    #[allow(unused)]
    pub fn get_linear_acceleration_unit(
        &mut self,
    ) -> Result<registers::unit_sel::Acceleration, Error> {
        if self.acceleration_unit.is_some() {
            return Ok(self.acceleration_unit.as_ref().unwrap().clone());
        }

        self.acceleration_unit = Some(self.get_register_value(registers::unit_sel::ADDRESS)?);

        Ok(self.acceleration_unit.as_ref().unwrap().clone())
    }

    #[allow(unused)]
    pub fn set_temperature_unit(
        &mut self,
        temperature_unit: registers::unit_sel::Temperature,
    ) -> Result<(), Error> {
        self.temperature_unit = Some(temperature_unit.clone());
        self.set_register_value(registers::unit_sel::ADDRESS, temperature_unit)
    }

    #[allow(unused)]
    pub fn get_temperature_unit(&mut self) -> Result<registers::unit_sel::Temperature, Error> {
        if self.temperature_unit.is_some() {
            return Ok(self.temperature_unit.as_ref().unwrap().clone());
        }

        self.temperature_unit = Some(self.get_register_value(registers::unit_sel::ADDRESS)?);

        Ok(self.temperature_unit.as_ref().unwrap().clone())
    }

    #[allow(unused)]
    pub fn set_power_mode(
        &mut self,
        power_mode: registers::power_mode::PowerMode,
    ) -> Result<(), Error> {
        self.set_register_value(registers::power_mode::ADDRESS, power_mode)
    }

    #[allow(unused)]
    pub fn get_power_mode(&mut self) -> Result<registers::power_mode::PowerMode, Error> {
        self.get_register_value(registers::power_mode::ADDRESS)
    }

    #[allow(unused)]
    pub fn set_operating_mode(
        &mut self,
        operating_mode: registers::opr_mode::OperatingMode,
    ) -> Result<(), Error> {
        // Gets the operating mode.
        let current_operating_mode: registers::opr_mode::OperatingMode =
            self.get_operating_mode()?;

        // Gets the duration of the mode change, also immediately checks if the mode change
        //  is valid, and if not, it will return an error.
        let duration: std::time::Duration = if current_operating_mode
            == registers::opr_mode::OperatingMode::ConfigMode
            && operating_mode != registers::opr_mode::OperatingMode::ConfigMode
        {
            std::time::Duration::from_millis(7)
        } else if operating_mode == registers::opr_mode::OperatingMode::ConfigMode
            && current_operating_mode != registers::opr_mode::OperatingMode::ConfigMode
        {
            std::time::Duration::from_millis(19)
        } else {
            return Err(Error::General(
                "Can only set operating mode from and to config.",
            ));
        };

        // Sets the register value.
        self.set_register_value(registers::opr_mode::ADDRESS, operating_mode)?;

        // Sleeps the required duration for the operation to be performed.
        std::thread::sleep(duration);

        Ok(())
    }

    #[allow(unused)]
    pub fn get_system_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::SysCalibStat, Error> {
        self.get_register_value(registers::calib_stat::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_gyroscope_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::GyrCalibStat, Error> {
        self.get_register_value(registers::calib_stat::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_accelerometer_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::AccCalibStat, Error> {
        self.get_register_value(registers::calib_stat::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_magnetometer_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::MagCalibStat, Error> {
        self.get_register_value(registers::calib_stat::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_operating_mode(&mut self) -> Result<registers::opr_mode::OperatingMode, Error> {
        self.get_register_value(registers::opr_mode::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_accelerometer_radius(&mut self) -> Result<i16, Error> {
        self.read_i16(registers::ACC_RADIUS_LSB)
    }

    #[allow(unused)]
    pub fn set_accelerometer_radius(&mut self, acceleration_radius: i16) -> Result<(), Error> {
        self.write_i16(registers::ACC_RADIUS_LSB, acceleration_radius)
    }

    #[allow(unused)]
    pub fn get_gyroscope_x_offset(&mut self) -> Result<i16, Error> {
        self.read_i16(registers::GYR_X_OFFSET_LSB)
    }

    #[allow(unused)]
    pub fn set_gyroscope_x_offset(&mut self, gyroscope_x_offset: i16) -> Result<(), Error> {
        self.write_i16(registers::GYR_X_OFFSET_LSB, gyroscope_x_offset)
    }

    #[allow(unused)]
    pub fn get_gyroscope_y_offset(&mut self) -> Result<i16, Error> {
        self.read_i16(registers::GYR_Y_OFFSET_LSB)
    }

    #[allow(unused)]
    pub fn set_gyroscope_y_offset(&mut self, gyroscope_y_offset: i16) -> Result<(), Error> {
        self.write_i16(registers::GYR_Y_OFFSET_LSB, gyroscope_y_offset)
    }

    #[allow(unused)]
    pub fn get_gyroscope_z_offset(&mut self) -> Result<i16, Error> {
        self.read_i16(registers::GYR_Z_OFFSET_LSB)
    }

    #[allow(unused)]
    pub fn set_gyroscope_z_offset(&mut self, gyroscope_z_offset: i16) -> Result<(), Error> {
        self.write_i16(registers::GYR_Z_OFFSET_LSB, gyroscope_z_offset)
    }

    #[allow(unused)]
    pub fn get_acceleration_x_offset(&mut self) -> Result<i16, Error> {
        self.read_i16(registers::ACC_OFFSET_X_LSB)
    }

    #[allow(unused)]
    pub fn set_acceleration_x_offset(&mut self, acceleration_x_offset: i16) -> Result<(), Error> {
        self.write_i16(registers::ACC_OFFSET_X_LSB, acceleration_x_offset)
    }

    #[allow(unused)]
    pub fn get_acceleration_y_offset(&mut self) -> Result<i16, Error> {
        self.read_i16(registers::ACC_OFFSET_Y_LSB)
    }

    #[allow(unused)]
    pub fn set_acceleration_y_offset(&mut self, acceleration_y_offset: i16) -> Result<(), Error> {
        self.write_i16(registers::ACC_OFFSET_Y_LSB, acceleration_y_offset)
    }

    #[allow(unused)]
    pub fn get_acceleration_z_offset(&mut self) -> Result<i16, Error> {
        self.read_i16(registers::ACC_OFFSET_Z_LSB)
    }

    #[allow(unused)]
    pub fn set_acceleration_z_offset(&mut self, acceleration_z_offset: i16) -> Result<(), Error> {
        self.write_i16(registers::ACC_OFFSET_Z_LSB, acceleration_z_offset)
    }

    #[allow(unused)]
    pub fn get_euler_angles(&mut self) -> Result<Vector3<f64>, Error> {
        let unit: registers::unit_sel::EulerAngles = self.get_euler_angles_unit()?;

        let x: u16 = self.read_u16(registers::EUL_PITCH_LSB)?;
        let y: u16 = self.read_u16(registers::EUL_ROLL_LSB)?;

        let euler_angles: Vector3<f64> = Vector3::<f64>::new(x as f64, y as f64, 0.0);

        match unit {
            registers::unit_sel::EulerAngles::Degrees => Ok(euler_angles / 16.0),
            registers::unit_sel::EulerAngles::Radians => Ok(euler_angles / 900.0),
        }
    }

    #[allow(unused)]
    pub fn get_linear_acceleration(&mut self) -> Result<Vector3<f64>, Error> {
        let unit: registers::unit_sel::Acceleration = self.get_acceleration_unit()?;

        let x: u16 = self.read_u16(registers::LIA_DATA_X_LSB)?;
        let y: u16 = self.read_u16(registers::LIA_DATA_Y_LSB)?;
        let z: u16 = self.read_u16(registers::LIA_DATA_Z_LSB)?;

        let acceleration: Vector3<f64> = Vector3::<f64>::new(x as f64, y as f64, z as f64);

        match unit {
            registers::unit_sel::Acceleration::MilliG => Ok(acceleration / 1.0),
            registers::unit_sel::Acceleration::Mpss => Ok(acceleration / 100.0),
        }
    }

    #[allow(unused)]
    pub fn get_gravity(&mut self) -> Result<Vector3<f64>, Error> {
        let unit: registers::unit_sel::Acceleration = self.get_acceleration_unit()?;

        let x: u16 = self.read_u16(registers::GRV_DATA_X_LSB)?;
        let y: u16 = self.read_u16(registers::GRV_DATA_Y_LSB)?;
        let z: u16 = self.read_u16(registers::GRV_DATA_Z_LSB)?;

        let acceleration: Vector3<f64> = Vector3::<f64>::new(x as f64, y as f64, z as f64);

        match unit {
            registers::unit_sel::Acceleration::MilliG => Ok(acceleration / 1.0),
            registers::unit_sel::Acceleration::Mpss => Ok(acceleration / 100.0),
        }
    }

    #[allow(unused)]
    pub fn get_temperature(&mut self) -> Result<f64, Error> {
        let unit: registers::unit_sel::Temperature = self.get_temperature_unit()?;

        let temperature: f64 = self.read_u16(registers::TEMP)? as f64;

        match unit {
            registers::unit_sel::Temperature::Celsius => Ok(temperature / 1.0),
            registers::unit_sel::Temperature::Fahrenheit => Ok(temperature * 2.0),
        }
    }
}

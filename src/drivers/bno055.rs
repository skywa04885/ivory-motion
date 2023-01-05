use nalgebra::{Quaternion, Vector, Vector3, Vector4};

pub mod registers;

#[derive(Debug)]
pub struct Identifiers {
    pub software_revision_id: u16,
    pub chip_id: u8,
    pub accelerometer_chip_id: u8,
    pub gyroscope_chip_id: u8,
}

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
pub struct Driver {
    i2c: rppal::i2c::I2c,
    i2c_slave_address: u16,
    euler_angles_unit: Option<registers::unit_sel::EulerAngles>,
    acceleration_unit: Option<registers::unit_sel::Acceleration>,
    temperature_unit: Option<registers::unit_sel::Temperature>,
}

impl Identifiers {
    pub fn new(
        software_revision_id: u16,
        chip_id: u8,
        accelerometer_chip_id: u8,
        gyroscope_chip_id: u8,
    ) -> Self {
        Self {
            software_revision_id,
            chip_id,
            accelerometer_chip_id,
            gyroscope_chip_id,
        }
    }
}

impl Driver {
    #[allow(unused)]
    pub fn new(i2c_slave_address: u16, i2c: rppal::i2c::I2c) -> Self {
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
        self.i2c.set_slave_address(self.i2c_slave_address)?;

        let write_buffer: [u8; 1] = [address];
        let mut read_buffer: [u8; 1] = [0; 1];

        self.i2c.write_read(&write_buffer, &mut read_buffer)?;

        let value: u8 = u8::from_le_bytes(read_buffer);

        Ok(value)
    }

    #[allow(unused)]
    pub fn write_u8(&mut self, address: u8, value: u8) -> Result<(), Error> {
        self.i2c.set_slave_address(self.i2c_slave_address)?;

        let value_buffer: [u8; 1] = value.to_le_bytes();
        let write_buffer: [u8; 2] = [address, value_buffer[0]];

        self.i2c.write(&write_buffer)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn read_u16(&mut self, address: u8) -> Result<u16, Error> {
        self.i2c.set_slave_address(self.i2c_slave_address)?;

        let write_buffer: [u8; 1] = [address];
        let mut read_buffer: [u8; 2] = [0; 2];

        self.i2c.write_read(&write_buffer, &mut read_buffer)?;

        let value: u16 = u16::from_le_bytes(read_buffer);

        Ok(value)
    }

    #[allow(unused)]
    pub fn write_u16(&mut self, address: u8, value: u16) -> Result<(), Error> {
        self.i2c.set_slave_address(self.i2c_slave_address)?;

        let value_bytes: [u8; 2] = value.to_le_bytes();
        let write_buffer: [u8; 3] = [address, value_bytes[0], value_bytes[1]];

        self.i2c.write(&write_buffer)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn read_i16(&mut self, address: u8) -> Result<i16, Error> {
        self.i2c.set_slave_address(self.i2c_slave_address)?;

        let write_buffer: [u8; 1] = [address];
        let mut read_buffer: [u8; 2] = [0; 2];

        self.i2c.write_read(&write_buffer, &mut read_buffer)?;

        let value: i16 = i16::from_le_bytes(read_buffer);

        Ok(value)
    }

    #[allow(unused)]
    pub fn write_i16(&mut self, address: u8, value: i16) -> Result<(), Error> {
        self.i2c.set_slave_address(self.i2c_slave_address)?;

        let value_bytes: [u8; 2] = value.to_le_bytes();
        let write_buffer: [u8; 3] = [address, value_bytes[0], value_bytes[1]];

        self.i2c.write(&write_buffer)?;

        Ok(())
    }

    #[allow(unused)]
    pub fn set_acceleration_unit(
        &mut self,
        acceleration_unit: registers::unit_sel::Acceleration,
    ) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.acceleration_unit = Some(acceleration_unit.clone());
        self.set_register_value(registers::unit_sel::ADDRESS, acceleration_unit)
    }

    #[allow(unused)]
    pub fn get_acceleration_unit(&mut self) -> Result<registers::unit_sel::Acceleration, Error> {
        assert!(self.get_page_id()? == 0);

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
        assert!(self.get_page_id()? == 0);

        self.set_register_value(registers::unit_sel::ADDRESS, angular_rate_unit)
    }

    #[allow(unused)]
    pub fn get_angular_rate_unit(&mut self) -> Result<registers::unit_sel::AngularRate, Error> {
        assert!(self.get_page_id()? == 0);

        self.get_register_value(registers::unit_sel::ADDRESS)
    }

    #[allow(unused)]
    pub fn set_euler_angles_unit(
        &mut self,
        euler_angles_unit: registers::unit_sel::EulerAngles,
    ) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.euler_angles_unit = Some(euler_angles_unit.clone());
        self.set_register_value(registers::unit_sel::ADDRESS, euler_angles_unit)
    }

    #[allow(unused)]
    pub fn get_euler_angles_unit(&mut self) -> Result<registers::unit_sel::EulerAngles, Error> {
        assert!(self.get_page_id()? == 0);

        if self.euler_angles_unit.is_some() {
            return Ok(self.euler_angles_unit.as_ref().unwrap().clone());
        }

        self.euler_angles_unit = Some(self.get_register_value(registers::unit_sel::ADDRESS)?);

        Ok(self.euler_angles_unit.as_ref().unwrap().clone())
    }

    #[allow(unused)]
    pub fn set_temperature_unit(
        &mut self,
        temperature_unit: registers::unit_sel::Temperature,
    ) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.temperature_unit = Some(temperature_unit.clone());
        self.set_register_value(registers::unit_sel::ADDRESS, temperature_unit)
    }

    #[allow(unused)]
    pub fn get_temperature_unit(&mut self) -> Result<registers::unit_sel::Temperature, Error> {
        assert!(self.get_page_id()? == 0);

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
        self.set_register_value(registers::power_mode::ADDRESS, power_mode)?;

        std::thread::sleep(std::time::Duration::from_millis(10));

        Ok(())
    }

    #[allow(unused)]
    pub fn get_power_mode(&mut self) -> Result<registers::power_mode::PowerMode, Error> {
        assert!(self.get_page_id()? == 0);

        self.get_register_value(registers::power_mode::ADDRESS)
    }

    #[allow(unused)]
    pub fn set_operating_mode(
        &mut self,
        operating_mode: registers::opr_mode::OperatingMode,
    ) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.set_register_value(registers::opr_mode::ADDRESS, operating_mode)?;

        std::thread::sleep(std::time::Duration::from_millis(400));

        Ok(())
    }

    #[allow(unused)]
    pub fn get_system_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::SysCalibStat, Error> {
        assert!(self.get_page_id()? == 0);

        self.get_register_value(registers::calib_stat::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_gyroscope_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::GyrCalibStat, Error> {
        assert!(self.get_page_id()? == 0);

        self.get_register_value(registers::calib_stat::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_accelerometer_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::AccCalibStat, Error> {
        assert!(self.get_page_id()? == 0);

        self.get_register_value(registers::calib_stat::ADDRESS)
   }

    #[allow(unused)]
    pub fn get_magnetometer_calibration_status(
        &mut self,
    ) -> Result<registers::calib_stat::MagCalibStat, Error> {
        assert!(self.get_page_id()? == 0);

        self.get_register_value(registers::calib_stat::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_operating_mode(&mut self) -> Result<registers::opr_mode::OperatingMode, Error> {
        assert!(self.get_page_id()? == 0);

        self.get_register_value(registers::opr_mode::ADDRESS)
    }

    #[allow(unused)]
    pub fn get_accelerometer_radius(&mut self) -> Result<i16, Error> {
        assert!(self.get_page_id()? == 0);

        self.read_i16(registers::ACC_RADIUS_LSB)
    }

    #[allow(unused)]
    pub fn set_accelerometer_radius(&mut self, acceleration_radius: i16) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.write_i16(registers::ACC_RADIUS_LSB, acceleration_radius)
    }

    #[allow(unused)]
    pub fn get_gyroscope_x_offset(&mut self) -> Result<i16, Error> {
        assert!(self.get_page_id()? == 0);

        self.read_i16(registers::GYR_X_OFFSET_LSB)
    }

    #[allow(unused)]
    pub fn set_gyroscope_x_offset(&mut self, gyroscope_x_offset: i16) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.write_i16(registers::GYR_X_OFFSET_LSB, gyroscope_x_offset)
    }

    #[allow(unused)]
    pub fn get_gyroscope_y_offset(&mut self) -> Result<i16, Error> {
        assert!(self.get_page_id()? == 0);

        self.read_i16(registers::GYR_Y_OFFSET_LSB)
    }

    #[allow(unused)]
    pub fn set_gyroscope_y_offset(&mut self, gyroscope_y_offset: i16) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.write_i16(registers::GYR_Y_OFFSET_LSB, gyroscope_y_offset)
    }

    #[allow(unused)]
    pub fn get_gyroscope_z_offset(&mut self) -> Result<i16, Error> {
        assert!(self.get_page_id()? == 0);

        self.read_i16(registers::GYR_Z_OFFSET_LSB)
    }

    #[allow(unused)]
    pub fn set_gyroscope_z_offset(&mut self, gyroscope_z_offset: i16) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.write_i16(registers::GYR_Z_OFFSET_LSB, gyroscope_z_offset)
    }

    #[allow(unused)]
    pub fn get_acceleration_x_offset(&mut self) -> Result<i16, Error> {
        assert!(self.get_page_id()? == 0);

        self.read_i16(registers::ACC_OFFSET_X_LSB)
    }

    #[allow(unused)]
    pub fn set_acceleration_x_offset(&mut self, acceleration_x_offset: i16) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.write_i16(registers::ACC_OFFSET_X_LSB, acceleration_x_offset)
    }

    #[allow(unused)]
    pub fn get_acceleration_y_offset(&mut self) -> Result<i16, Error> {
        assert!(self.get_page_id()? == 0);

        self.read_i16(registers::ACC_OFFSET_Y_LSB)
    }

    #[allow(unused)]
    pub fn set_acceleration_y_offset(&mut self, acceleration_y_offset: i16) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.write_i16(registers::ACC_OFFSET_Y_LSB, acceleration_y_offset)
    }

    #[allow(unused)]
    pub fn get_acceleration_z_offset(&mut self) -> Result<i16, Error> {
        assert!(self.get_page_id()? == 0);

        self.read_i16(registers::ACC_OFFSET_Z_LSB)
    }

    #[allow(unused)]
    pub fn set_acceleration_z_offset(&mut self, acceleration_z_offset: i16) -> Result<(), Error> {
        assert!(self.get_page_id()? == 0);

        self.write_i16(registers::ACC_OFFSET_Z_LSB, acceleration_z_offset)
    }

    #[allow(unused)]
    pub fn get_euler_angles(&mut self) -> Result<Vector3<f64>, Error> {
        assert!(self.get_page_id()? == 0);

        let unit: registers::unit_sel::EulerAngles = self.get_euler_angles_unit()?;

        let (pitch, roll, yaw) = (
            self.read_i16(registers::EUL_PITCH_LSB)?,
            self.read_i16(registers::EUL_ROLL_LSB)?,
            self.read_i16(registers::EUL_YAW_LSB)?,
        );

        let euler_angles: Vector3<f64> = Vector3::<f64>::new(pitch as f64, roll as f64, yaw as f64);

        match unit {
            registers::unit_sel::EulerAngles::Degrees => Ok(euler_angles / 16.0),
            registers::unit_sel::EulerAngles::Radians => Ok(euler_angles / 900.0),
        }
    }

    #[allow(unused)]
    pub fn get_linear_acceleration(&mut self) -> Result<Vector3<f64>, Error> {
        assert!(self.get_page_id()? == 0);

        let unit: registers::unit_sel::Acceleration = self.get_acceleration_unit()?;

        let (x, y, z) = (
            self.read_i16(registers::LIA_DATA_X_LSB)?,
            self.read_i16(registers::LIA_DATA_Y_LSB)?,
            self.read_i16(registers::LIA_DATA_Z_LSB)?,
        );

        let acceleration: Vector3<f64> = Vector3::<f64>::new(x as f64, y as f64, z as f64);

        match unit {
            registers::unit_sel::Acceleration::MilliG => Ok(acceleration / 1.0),
            registers::unit_sel::Acceleration::Mpss => Ok(acceleration / 100.0),
        }
    }

    #[allow(unused)]
    pub fn get_gravity(&mut self) -> Result<Vector3<f64>, Error> {
        assert!(self.get_page_id()? == 0);

        let unit: registers::unit_sel::Acceleration = self.get_acceleration_unit()?;

        let (x, y, z) = (
            self.read_i16(registers::GRV_DATA_X_LSB)?,
            self.read_i16(registers::GRV_DATA_Y_LSB)?,
            self.read_i16(registers::GRV_DATA_Z_LSB)?,
        );

        let acceleration: Vector3<f64> = Vector3::<f64>::new(x as f64, y as f64, z as f64);

        match unit {
            registers::unit_sel::Acceleration::MilliG => Ok(acceleration / 1.0),
            registers::unit_sel::Acceleration::Mpss => Ok(acceleration / 100.0),
        }
    }

    #[allow(unused)]
    pub fn get_acceleration(&mut self) -> Result<Vector3<f64>, Error> {
        assert!(self.get_page_id()? == 0);

        let unit: registers::unit_sel::Acceleration = self.get_acceleration_unit()?;

        let (x, y, z) = (
            self.read_i16(registers::ACC_DATA_X_LSB)?,
            self.read_i16(registers::ACC_DATA_Y_LSB)?,
            self.read_i16(registers::ACC_DATA_Z_LSB)?,
        );
        
        let acceleration: Vector3<f64> = Vector3::<f64>::new(x as f64, y as f64, z as f64);

        match unit {
            registers::unit_sel::Acceleration::MilliG => Ok(acceleration / 1.0),
            registers::unit_sel::Acceleration::Mpss => Ok(acceleration / 100.0),
        }
    }

    #[allow(unused)]
    pub fn get_quaternion(&mut self) -> Result<Quaternion<f64>, Error> {
        assert!(self.get_page_id()? == 0);

        let (w, x, y, z) = (
            self.read_i16(registers::QUA_DATA_W_LSB)?,
            self.read_i16(registers::QUA_DATA_X_LSB)?,
            self.read_i16(registers::QUA_DATA_Y_LSB)?,
            self.read_i16(registers::QUA_DATA_Z_LSB)?,
        );

        let mut quaternion: Vector4<f64> =
            Vector4::<f64>::new(w as f64, x as f64, y as f64, z as f64);

        quaternion /= f64::powf(2.0, 14.0);

        Ok(Quaternion::<f64>::from_vector(quaternion))
    }

    #[allow(unused)]
    pub fn get_temperature(&mut self) -> Result<f64, Error> {
        assert!(self.get_page_id()? == 0);

        let unit: registers::unit_sel::Temperature = self.get_temperature_unit()?;

        let temperature: f64 = self.read_i16(registers::TEMP)? as f64;

        match unit {
            registers::unit_sel::Temperature::Celsius => Ok(temperature / 1.0),
            registers::unit_sel::Temperature::Fahrenheit => Ok(temperature * 2.0),
        }
    }

    #[allow(unused)]
    pub fn get_software_revision_id(&mut self) -> Result<u16, Error> {
        assert!(self.get_page_id()? == 0);
        self.read_u16(registers::SW_REV_ID_LSB)
    }

    #[allow(unused)]
    pub fn get_chip_id(&mut self) -> Result<u8, Error> {
        assert!(self.get_page_id()? == 0);
        self.read_u8(registers::CHIP_ID)
    }

    #[allow(unused)]
    pub fn get_accelerometer_chip_id(&mut self) -> Result<u8, Error> {
        assert!(self.get_page_id()? == 0);
        self.read_u8(registers::ACC_ID)
    }

    #[allow(unused)]
    pub fn get_gyroscope_chip_id(&mut self) -> Result<u8, Error> {
        assert!(self.get_page_id()? == 0);
        self.read_u8(registers::GYR_ID)
    }

    #[allow(unused)]
    pub fn get_identifiers(&mut self) -> Result<Identifiers, Error> {
        Ok(Identifiers::new(
            self.get_software_revision_id()?,
            self.get_chip_id()?,
            self.get_accelerometer_chip_id()?,
            self.get_gyroscope_chip_id()?,
        ))
    }

    #[allow(unused)]
    pub fn get_page_id(&mut self) -> Result<u8, Error> {
        self.read_u8(registers::PAGE_ID)
    }

    #[allow(unused)]
    pub fn reset(&mut self) -> Result<(), Error> {
        self.set_register_value(
            registers::sys_trigger::ADDRESS,
            registers::sys_trigger::RstSys::Reset,
        )?;

        std::thread::sleep(std::time::Duration::from_millis(1500));

        Ok(())
    }
}

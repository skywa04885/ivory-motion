use nalgebra::Vector3;

pub const ABSOLUTE_ORIENTATION_ROUTING_KEY: &'static str = "sensors.imu.absolute_orientation";

#[derive(serde::Serialize)]
pub struct AbsoluteOrientationMessage {
    absolute_orientation: Vector3<f64>,
}

impl AbsoluteOrientationMessage {
    pub fn new(absolute_orientation: Vector3<f64>) -> Self {
        Self {
            absolute_orientation,
        }
    }
}

pub const ANGULAR_VELOCITY_ROUTING_KEY: &'static str = "sensors.imu.angular_velocity";

#[derive(serde::Serialize)]
pub struct AngularVelocityMessage {
    angular_velocity: Vector3<f64>,
}

impl AngularVelocityMessage {
    pub fn new(angular_velocity: Vector3<f64>) -> Self {
        Self { angular_velocity }
    }
}

pub const ACCELERATION_ROUTING_KEY: &'static str = "sensors.imu.acceleration";

#[derive(serde::Serialize)]
pub struct AccelerationMessage {
    acceleration: Vector3<f64>,
}

impl AccelerationMessage {
    pub fn new(acceleration: Vector3<f64>) -> Self {
        Self { acceleration }
    }
}

pub const LINEAR_ACCELERATION_ROUTING_KEY: &'static str = "sensors.imu.linear_acceleration";

#[derive(serde::Serialize)]
pub struct LinearAccelerationMessage {
    linear_acceleration: Vector3<f64>,
}

impl LinearAccelerationMessage {
    pub fn new(linear_acceleration: Vector3<f64>) -> Self {
        Self {
            linear_acceleration,
        }
    }
}

pub const GRAVITY_ROUTING_KEY: &'static str = "sensors.imu.gravity";

#[derive(serde::Serialize)]
pub struct GravityMessage {
    gravity: Vector3<f64>,
}

impl GravityMessage {
    pub fn new(gravity: Vector3<f64>) -> Self {
        Self { gravity }
    }
}

pub const MAGNETIC_FIELD_ROUTING_KEY: &'static str = "sensors.imu.magnetic_field";

#[derive(serde::Serialize)]
pub struct MagneticFieldMessage {
    magnetic_field: Vector3<f64>,
}

impl MagneticFieldMessage {
    pub fn new(magnetic_field: Vector3<f64>) -> Self {
        Self { magnetic_field }
    }
}

pub const TEMPERATURE_ROUTING_KEY: &'static str = "sensors.imu.temperature";

#[derive(serde::Serialize)]
pub struct TemperatureMessage {
    temperature: f64,
}

impl TemperatureMessage {
    pub fn new(temperature: f64) -> Self {
        Self { temperature }
    }
}

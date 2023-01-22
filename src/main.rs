use clap::Parser;
use log::{error, info};
use nalgebra::Vector3;
use std::io::BufRead;
use std::thread::spawn;

mod config;
mod drivers;
mod messages;

#[derive(Debug)]
enum MQThreadMessage {
    ImuEulerAngles(Vector3<f64>),
    ImuAngularVelocity(Vector3<f64>),
    ImuAcceleration(Vector3<f64>),
    ImuLinearAcceleration(Vector3<f64>),
    ImuGravity(Vector3<f64>),
    ImuMagneticField(Vector3<f64>),
    ImuTemperature(f64),
}

#[derive(Parser, Debug, Clone)]
#[command(author="Luke A.C.A. Rieff", version, about="Kinematics Service for Unicorn", long_about = None)]
struct Arguments {
    #[clap(long, default_value = "./env/config.toml")]
    config: String,
    command: String,
}

fn spawn_mq_thread(
    config: config::Config,
    mut mq_thread_command_rx: tokio::sync::mpsc::Receiver<MQThreadMessage>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let runtime: tokio::runtime::Runtime = tokio::runtime::Runtime::new().unwrap();
        runtime.block_on(async move {
            // Constructs the connection URI for the RabbitMQ server.
            let lapin_connection_uri: String = if config.rabbitmq.username.is_none() {
                info!("Username appears to be empty, connecting without authentication.");
                format!(
                    "amqp://{}:{}",
                    config.rabbitmq.hostname, config.rabbitmq.port
                )
            } else {
                info!("Username appears to be set, connecting using authentication.");
                format!(
                    "amqp://{}:{}@{}:{}",
                    config.rabbitmq.username.as_ref().unwrap(),
                    config.rabbitmq.password.as_ref().unwrap(),
                    config.rabbitmq.hostname,
                    config.rabbitmq.port,
                )
            };

            // Constructs the properties for the lapin connection.
            let lapin_connection_properties: lapin::ConnectionProperties =
                lapin::ConnectionProperties::default()
                    .with_executor(tokio_executor_trait::Tokio::current())
                    .with_reactor(tokio_reactor_trait::Tokio);

            // Connects to the RabbitMQ server.
            info!(
                "Connecting to RabbitMQ with uri: \"{}\"",
                lapin_connection_uri
            );
            let lapin_connection: lapin::Connection =
                lapin::Connection::connect(&lapin_connection_uri, lapin_connection_properties)
                    .await
                    .unwrap();
            info!(
                "Connected to RabbitMQ with uri: \"{}\"",
                lapin_connection_uri
            );

            // Creates the channel.
            let lapin_channel: lapin::Channel = lapin_connection.create_channel().await.unwrap();

            // Declares the topic exchange for the current robot (to separate the data from the rest).
            info!("Using exchange: \"{}\"", config.rabbitmq.exchange);
            lapin_channel
                .exchange_declare(
                    &config.rabbitmq.exchange,
                    lapin::ExchangeKind::Topic,
                    lapin::options::ExchangeDeclareOptions::default(),
                    lapin::types::FieldTable::default(),
                )
                .await
                .unwrap();

            // Processes the commands from the channel.
            while let Some(command) = mq_thread_command_rx.recv().await {
                let time: u128 = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_millis();
                match command {
                    // Absolute orientation.
                    MQThreadMessage::ImuEulerAngles(absolute_orientation) => {
                        let message: messages::sensors::bno055::EulerAnglesMessage =
                            messages::sensors::bno055::EulerAnglesMessage::new(
                                absolute_orientation,
                                time,
                            );
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::EULER_ANGLES_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Angular velocity.
                    MQThreadMessage::ImuAngularVelocity(angular_velocity) => {
                        let message: messages::sensors::bno055::LinearAccelerationMessage =
                            messages::sensors::bno055::LinearAccelerationMessage::new(
                                angular_velocity,
                                time,
                            );
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::ANGULAR_VELOCITY_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Acceleration.
                    MQThreadMessage::ImuAcceleration(acceleration) => {
                        let message: messages::sensors::bno055::AccelerationMessage =
                            messages::sensors::bno055::AccelerationMessage::new(acceleration);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::ACCELERATION_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Linear acceleration.
                    MQThreadMessage::ImuLinearAcceleration(linear_acceleration) => {
                        let message: messages::sensors::bno055::LinearAccelerationMessage =
                            messages::sensors::bno055::LinearAccelerationMessage::new(
                                linear_acceleration,
                                time,
                            );
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::LINEAR_ACCELERATION_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Gravity.
                    MQThreadMessage::ImuGravity(gravity) => {
                        let message: messages::sensors::bno055::GravityMessage =
                            messages::sensors::bno055::GravityMessage::new(gravity, time);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::GRAVITY_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Magnetic field.
                    MQThreadMessage::ImuMagneticField(magnetic_field) => {
                        let message: messages::sensors::bno055::MagneticFieldMessage =
                            messages::sensors::bno055::MagneticFieldMessage::new(magnetic_field);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::MAGNETIC_FIELD_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Temperature.
                    MQThreadMessage::ImuTemperature(temperature) => {
                        let message: messages::sensors::bno055::TemperatureMessage =
                            messages::sensors::bno055::TemperatureMessage::new(temperature);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::TEMPERATURE_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                };
            }
        });
    })
}

/// Performs the BNO055 driver setup.
fn bno055_driver_setup(
    config: &config::Config,
    driver: &mut drivers::bno055::Driver,
) -> Result<(), drivers::bno055::Error> {
    // Resets the system, and sets the power mode to normal.
    driver.reset()?;
    driver.set_power_mode(drivers::bno055::registers::power_mode::PowerMode::Normal)?;

    // Sets the units.
    driver.set_acceleration_unit(drivers::bno055::registers::unit_sel::Acceleration::Mpss)?;
    driver.set_temperature_unit(drivers::bno055::registers::unit_sel::Temperature::Celsius)?;
    driver.set_angular_rate_unit(drivers::bno055::registers::unit_sel::AngularRate::Rps)?;
    driver.set_euler_angles_unit(drivers::bno055::registers::unit_sel::EulerAngles::Radians)?;

    // Loads the values from calibration.
    driver.set_gyroscope_x_offset(config.bno055.gyroscope_offset_x)?;
    driver.set_gyroscope_y_offset(config.bno055.gyroscope_offset_y)?;
    driver.set_gyroscope_z_offset(config.bno055.gyroscope_offset_z)?;
    driver.set_acceleration_x_offset(config.bno055.accelerometer_offset_x)?;
    driver.set_acceleration_y_offset(config.bno055.accelerometer_offset_y)?;
    driver.set_acceleration_z_offset(config.bno055.accelerometer_offset_z)?;
    driver.set_accelerometer_radius(160)?;

    // Changes the operating mode to inertial measurement unit.
    driver.set_operating_mode(drivers::bno055::registers::opr_mode::OperatingMode::Imu)?;

    Ok(())
}

fn spawn_imu_thread(
    arguments: Arguments,
    mq_thread_command_tx: std::sync::Arc<tokio::sync::mpsc::Sender<MQThreadMessage>>,
    mut bno055_driver: drivers::bno055::Driver,
    mut bno055_interrupt_pin: rppal::gpio::InputPin,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || loop {
        let euler_angles: Vector3<f64> = match bno055_driver.get_euler_angles() {
            Ok(euler_angles) => euler_angles,
            Err(error) => {
                error!("Failed to read euler angles: {:?}", error);
                return;
            }
        };

        let linear_acceleration: Vector3<f64> = match bno055_driver.get_linear_acceleration() {
            Ok(linear_acceleration) => linear_acceleration,
            Err(error) => {
                error!("Failed to read linear acceleration: {:?}", error);
                return;
            }
        };

        let gravity: Vector3<f64> = match bno055_driver.get_gravity() {
            Ok(gravity) => gravity,
            Err(error) => {
                error!("Failed to read gravity: {:?}", error);
                return;
            }
        };

        mq_thread_command_tx
            .blocking_send(MQThreadMessage::ImuEulerAngles(euler_angles))
            .unwrap();
        mq_thread_command_tx
            .blocking_send(MQThreadMessage::ImuLinearAcceleration(linear_acceleration))
            .unwrap();
        mq_thread_command_tx
            .blocking_send(MQThreadMessage::ImuGravity(gravity))
            .unwrap();

        std::thread::sleep(std::time::Duration::from_millis(200));
    })
}

/// Performs the calibration for the accelerometer.
fn calibrate_accelerometer(driver: &mut drivers::bno055::Driver) {
    if driver.get_accelerometer_calibration_status().unwrap()
        == drivers::bno055::registers::calib_stat::AccCalibStat::FullyCalibrated
    {
        info!("Accelerometer is fully calibrated, skipping procedure.");
        return;
    }

    info!("Move the module around and make sure you cover all perpendicular axises.");

    loop {
        let calib_status: drivers::bno055::registers::calib_stat::AccCalibStat =
            driver.get_accelerometer_calibration_status().unwrap();

        info!(
            "Current accelerometer calibration status: {:?}",
            calib_status
        );

        if calib_status == drivers::bno055::registers::calib_stat::AccCalibStat::FullyCalibrated {
            break;
        }

        std::thread::sleep(std::time::Duration::from_millis(700));
    }
}

/// Performs the calibration for the gyroscope.
fn calibrate_gyroscope(driver: &mut drivers::bno055::Driver) {
    if driver.get_gyroscope_calibration_status().unwrap()
        == drivers::bno055::registers::calib_stat::GyrCalibStat::FullyCalibrated
    {
        info!("Gyroscope is fully calibrated, skipping procedure.");
        return;
    }

    info!("Calibrating the gyroscope is quite simple, just place the module in a stable position.");

    loop {
        let calib_status: drivers::bno055::registers::calib_stat::GyrCalibStat =
            driver.get_gyroscope_calibration_status().unwrap();

        info!("Current gyroscope calibration status: {:?}", calib_status);

        if calib_status == drivers::bno055::registers::calib_stat::GyrCalibStat::FullyCalibrated {
            break;
        }

        std::thread::sleep(std::time::Duration::from_millis(700));
    }
}

fn main() {
    env_logger::init();

    // Parses the arguments and the configuration.
    let arguments: Arguments = Arguments::parse();
    let mut config: config::Config = config::Config::from_file(&arguments.config).unwrap();

    // Creates the handle for the I2C peripheral.
    let i2c: rppal::i2c::I2c = rppal::i2c::I2c::with_bus(3).unwrap();

    // Creates the BNO055 driver and performs it's setup.
    let mut bno055_interrupt_pin = rppal::gpio::Gpio::new()
        .unwrap()
        .get(4)
        .unwrap()
        .into_input_pulldown();
    bno055_interrupt_pin
        .set_interrupt(rppal::gpio::Trigger::RisingEdge)
        .unwrap();
    let mut bno055_driver: drivers::bno055::Driver =
        drivers::bno055::Driver::new(config.bno055.i2c_address, i2c);
    bno055_driver_setup(&config, &mut bno055_driver).unwrap();

    // Gets information about the BNO055.
    info!("{:?}", bno055_driver.get_identifiers().unwrap());
    info!(
        "System calibration: {:?}, Gyroscope calibration: {:?}, Accelerometer calibration: {:?}",
        bno055_driver.get_system_calibration_status(),
        bno055_driver.get_gyroscope_calibration_status(),
        bno055_driver.get_accelerometer_calibration_status()
    );

    // Handles the command.
    match arguments.command.as_str() {
        "calibrate" => {
            calibrate_accelerometer(&mut bno055_driver);
            calibrate_gyroscope(&mut bno055_driver);

            bno055_driver
                .set_operating_mode(drivers::bno055::registers::opr_mode::OperatingMode::ConfigMode)
                .unwrap();

            config.bno055.accelerometer_offset_x =
                bno055_driver.get_acceleration_x_offset().unwrap();
            config.bno055.accelerometer_offset_y =
                bno055_driver.get_acceleration_y_offset().unwrap();
            config.bno055.accelerometer_offset_z =
                bno055_driver.get_acceleration_z_offset().unwrap();
            config.bno055.gyroscope_offset_x = bno055_driver.get_gyroscope_x_offset().unwrap();
            config.bno055.gyroscope_offset_y = bno055_driver.get_gyroscope_y_offset().unwrap();
            config.bno055.gyroscope_offset_z = bno055_driver.get_gyroscope_z_offset().unwrap();

            info!(
                "Saving the updated BNO055 configuration to file: \"{}\"",
                arguments.config
            );
            config.to_file(&arguments.config).unwrap();
        }
        "run" => {
            let (mq_thread_command_tx, mq_thread_command_rx) =
                tokio::sync::mpsc::channel::<MQThreadMessage>(32);

            let mq_thread_command_tx: std::sync::Arc<tokio::sync::mpsc::Sender<MQThreadMessage>> =
                std::sync::Arc::new(mq_thread_command_tx);

            let (mq_thread, bno055_thread) = (
                spawn_mq_thread(config.clone(), mq_thread_command_rx),
                spawn_imu_thread(
                    arguments.clone(),
                    mq_thread_command_tx.clone(),
                    bno055_driver,
                    bno055_interrupt_pin,
                ),
            );

            mq_thread.join().unwrap();
            bno055_thread.join().unwrap();
        }
        command => error!("Cannot perform command: {}", command),
    }
}

use clap::Parser;
use log::{error, info};
use nalgebra::Vector3;
use std::io::BufRead;
use std::thread::spawn;

mod config;
mod drivers;
mod messages;

enum MQThreadMessage {
    ImuAbsoluteOrientation(Vector3<f64>),
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
                match command {
                    // Absolute orientation.
                    MQThreadMessage::ImuAbsoluteOrientation(absolute_orientation) => {
                        let message: messages::sensors::bno055::AbsoluteOrientationMessage =
                            messages::sensors::bno055::AbsoluteOrientationMessage::new(
                                absolute_orientation,
                            );
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &config.rabbitmq.exchange,
                                messages::sensors::bno055::ABSOLUTE_ORIENTATION_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Angular velocity.
                    MQThreadMessage::ImuAngularVelocity(angular_velocity) => {
                        let message: messages::sensors::bno055::AngularVelocityMessage =
                            messages::sensors::bno055::AngularVelocityMessage::new(
                                angular_velocity,
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
                            messages::sensors::bno055::GravityMessage::new(gravity);
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
    // Sets the units.
    driver.set_acceleration_unit(drivers::bno055::registers::unit_sel::Acceleration::Mpss)?;
    driver.set_temperature_unit(drivers::bno055::registers::unit_sel::Temperature::Celsius)?;
    driver.set_angular_rate_unit(drivers::bno055::registers::unit_sel::AngularRate::Rps)?;
    driver.set_euler_angles_unit(drivers::bno055::registers::unit_sel::EulerAngles::Radians)?;

    // Loads the values from calibration.
    driver.set_accelerometer_radius(config.bno055.accelerometer_radius)?;
    driver.set_gyroscope_x_offset(config.bno055.gyroscope_offset_x)?;
    driver.set_gyroscope_y_offset(config.bno055.gyroscope_offset_y)?;
    driver.set_gyroscope_z_offset(config.bno055.gyroscope_offset_z)?;
    driver.set_acceleration_x_offset(config.bno055.accelerometer_offset_x)?;
    driver.set_acceleration_y_offset(config.bno055.accelerometer_offset_y)?;
    driver.set_acceleration_z_offset(config.bno055.accelerometer_offset_z)?;

    // Changes the operating mode to inertial measurement unit.
    driver.set_operating_mode(drivers::bno055::registers::opr_mode::OperatingMode::Imu)?;

    Ok(())
}

fn spawn_imu_thread(
    arguments: Arguments,
    mq_thread_command_tx: std::sync::Arc<tokio::sync::mpsc::Sender<MQThreadMessage>>,
    bno055_driver: drivers::bno055::Driver,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || loop {
        std::thread::sleep(std::time::Duration::from_millis(50));
    })
}

/// Performs the calibration for the accelerometer.
fn calibrate_accelerometer(driver: &mut drivers::bno055::Driver) {
    if driver.get_accelerometer_calibration_status().unwrap()
        == drivers::bno055::registers::calib_stat::AccCalibStat::FullyCalibrated
    {
        info!("Accelerometer is fully calibrated, skipping procedure.");
    }

    let steps: [&'static str; 6] = [
        "Orient the module with the upside (where the chip is located) towards yourself.",
        "Rotate the module 90 degrees clockwise from the top perspective.",
        "Rotate the module another 90 degrees clockwise from the top perspective.",
        "Rotate the module 90 degrees counter-clockwise from the right-side perspective.",
        "Rotate the module 90 degrees clockwise from the front perspective.",
        "Rotate the module 90 degrees clockwise from the front perspective once more.",
    ];

    let stdin: std::io::Stdin = std::io::stdin();

    info!("We're about to calibrate the accelerometer, please do exactly what you are told.");

    loop {
        for step in steps {
            info!("{}", step);
            info!("Press enter once ready: ");

            let mut line = String::new();
            stdin.lock().read_line(&mut line).unwrap();

            for seconds_passed in 0..3 {
                info!("({} seconds remaining) Keep it steady!", 3 - seconds_passed);
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
        }

        let calib_status: drivers::bno055::registers::calib_stat::AccCalibStat =
            driver.get_accelerometer_calibration_status().unwrap();
        if calib_status == drivers::bno055::registers::calib_stat::AccCalibStat::FullyCalibrated {
            break;
        }

        info!(
            "Calibration failed, calibration status: \"{:?}\". Try again.",
            calib_status
        );
    }
}

/// Performs the calibration for the gyroscope.
fn calibrate_gyroscope(driver: &mut drivers::bno055::Driver) {
    if driver.get_gyroscope_calibration_status().unwrap()
        == drivers::bno055::registers::calib_stat::GyrCalibStat::FullyCalibrated
    {
        info!("Gyroscope is fully calibrated, skipping procedure.");
    }

    info!("Calibrating the gyroscope is quite simple, just place the module in a stable position.");

    loop {
        info!("Press enter once ready: ");

        for seconds_passed in 0..3 {
            println!("({} seconds remaining) Keep it stable!", 3 - seconds_passed);
            std::thread::sleep(std::time::Duration::from_secs(1));
        }

        let calib_status: drivers::bno055::registers::calib_stat::GyrCalibStat =
            driver.get_gyroscope_calibration_status().unwrap();
        if calib_status == drivers::bno055::registers::calib_stat::GyrCalibStat::FullyCalibrated {
            break;
        }

        info!(
            "Calibration failed, calibration status: \"{:?}\". Try again.",
            calib_status
        );
    }
}

fn main() {
    env_logger::init();

    // Parses the arguments and the configuration.
    let arguments: Arguments = Arguments::parse();
    let config: config::Config = config::Config::from_file(&arguments.config).unwrap();

    // Creates the handle for the I2C peripheral.
    let mut i2c: rppal::i2c::I2c = rppal::i2c::I2c::new().unwrap();

    // Creates the BNO055 driver and performs it's setup.
    let mut bno055_driver: drivers::bno055::Driver = drivers::bno055::Driver::new(0x28, &mut i2c);
    bno055_driver_setup(&config, &mut bno055_driver).unwrap();

    // Handles the command.
    match arguments.command.as_str() {
        "calibrate" => {
            calibrate_accelerometer(&mut bno055_driver);
            calibrate_gyroscope(&mut bno055_driver);
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
                ),
            );

            mq_thread.join().unwrap();
            bno055_thread.join().unwrap();
        }
        command => error!("Cannot perform command: {}", command),
    }
}

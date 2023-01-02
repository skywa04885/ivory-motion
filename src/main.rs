use std::thread::spawn;

use clap::Parser;
use log::{error, info};
use nalgebra::Vector3;

mod bno055;
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
    #[clap(long, default_value = "127.0.0.1")]
    rabbit_mq_hostname: String,
    #[clap(long, default_value = "5672")]
    rabbit_mq_port: u16,
    #[clap(long, default_value = "")]
    rabbit_mq_username: String,
    #[clap(long, default_value = "")]
    rabbit_mq_password: String,
    exchange: String,
}

fn spawn_mq_thread(
    arguments: Arguments,
    mut mq_thread_command_rx: tokio::sync::mpsc::Receiver<MQThreadMessage>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let runtime: tokio::runtime::Runtime = tokio::runtime::Runtime::new().unwrap();
        runtime.block_on(async move {
            // Constructs the connection URI for the RabbitMQ server.
            let lapin_connection_uri: String = if arguments.rabbit_mq_username.is_empty() {
                info!("Username appears to be empty, connecting without authentication.");
                format!(
                    "amqp://{}:{}",
                    arguments.rabbit_mq_hostname, arguments.rabbit_mq_port
                )
            } else {
                info!("Username appears to be set, connecting using authentication.");
                format!(
                    "amqp://{}:{}@{}:{}",
                    arguments.rabbit_mq_username,
                    arguments.rabbit_mq_password,
                    arguments.rabbit_mq_hostname,
                    arguments.rabbit_mq_port
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
            info!("Using exchange: \"{}\"", arguments.exchange);
            lapin_channel
                .exchange_declare(
                    &arguments.exchange,
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
                        let message: messages::sensors::imu::AbsoluteOrientationMessage =
                            messages::sensors::imu::AbsoluteOrientationMessage::new(
                                absolute_orientation,
                            );
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &arguments.exchange,
                                messages::sensors::imu::ABSOLUTE_ORIENTATION_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Angular velocity.
                    MQThreadMessage::ImuAngularVelocity(angular_velocity) => {
                        let message: messages::sensors::imu::AngularVelocityMessage =
                            messages::sensors::imu::AngularVelocityMessage::new(angular_velocity);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &arguments.exchange,
                                messages::sensors::imu::ANGULAR_VELOCITY_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Acceleration.
                    MQThreadMessage::ImuAcceleration(acceleration) => {
                        let message: messages::sensors::imu::AccelerationMessage =
                            messages::sensors::imu::AccelerationMessage::new(acceleration);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &arguments.exchange,
                                messages::sensors::imu::ACCELERATION_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Linear acceleration.
                    MQThreadMessage::ImuLinearAcceleration(linear_acceleration) => {
                        let message: messages::sensors::imu::LinearAccelerationMessage =
                            messages::sensors::imu::LinearAccelerationMessage::new(
                                linear_acceleration,
                            );
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &arguments.exchange,
                                messages::sensors::imu::LINEAR_ACCELERATION_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Gravity.
                    MQThreadMessage::ImuGravity(gravity) => {
                        let message: messages::sensors::imu::GravityMessage =
                            messages::sensors::imu::GravityMessage::new(gravity);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &arguments.exchange,
                                messages::sensors::imu::GRAVITY_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Magnetic field.
                    MQThreadMessage::ImuMagneticField(magnetic_field) => {
                        let message: messages::sensors::imu::MagneticFieldMessage =
                            messages::sensors::imu::MagneticFieldMessage::new(magnetic_field);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &arguments.exchange,
                                messages::sensors::imu::MAGNETIC_FIELD_ROUTING_KEY,
                                lapin::options::BasicPublishOptions::default(),
                                &encoded_message,
                                lapin::BasicProperties::default(),
                            )
                            .await
                            .unwrap();
                    }
                    // Temperature.
                    MQThreadMessage::ImuTemperature(temperature) => {
                        let message: messages::sensors::imu::TemperatureMessage =
                            messages::sensors::imu::TemperatureMessage::new(temperature);
                        let encoded_message: Vec<u8> = serde_json::to_vec(&message).unwrap();
                        lapin_channel
                            .basic_publish(
                                &arguments.exchange,
                                messages::sensors::imu::TEMPERATURE_ROUTING_KEY,
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

fn spawn_imu_thread(
    arguments: Arguments,
    mq_thread_command_tx: std::sync::Arc<tokio::sync::mpsc::Sender<MQThreadMessage>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut i2c: rppal::i2c::I2c = rppal::i2c::I2c::new().unwrap();
        let bno055: bno055::Driver = bno055::Driver::new(0x28, &mut i2c);

        loop {
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    })
}

fn main() {
    env_logger::init();

    let arguments: Arguments = Arguments::parse();

    let (mq_thread_command_tx, mq_thread_command_rx) =
        tokio::sync::mpsc::channel::<MQThreadMessage>(32);

    let mq_thread_command_tx: std::sync::Arc<tokio::sync::mpsc::Sender<MQThreadMessage>> =
        std::sync::Arc::new(mq_thread_command_tx);

    let (mq_thread, bno055_thread) = (
        spawn_mq_thread(arguments.clone(), mq_thread_command_rx),
        spawn_imu_thread(arguments.clone(), mq_thread_command_tx.clone()),
    );

    mq_thread.join().unwrap();
    bno055_thread.join().unwrap();
}

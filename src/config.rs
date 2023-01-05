#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    TomlDe(toml::de::Error),
    TomlSer(toml::ser::Error),
}

impl From<std::io::Error> for Error {
    fn from(value: std::io::Error) -> Self {
        Error::Io(value)
    }
}

impl From<toml::de::Error> for Error {
    fn from(value: toml::de::Error) -> Self {
        Error::TomlDe(value)
    }
}

impl From<toml::ser::Error> for Error {
    fn from(value: toml::ser::Error) -> Self {
        Error::TomlSer(value)
    }
}

#[derive(Debug, Clone, serde_derive::Deserialize, serde_derive::Serialize)]
pub struct Config {
    pub bno055: Bno055,
    pub rabbitmq: RabbitMQ,
}

#[derive(Debug, Clone, serde_derive::Deserialize, serde_derive::Serialize)]
pub struct Bno055 {
    pub i2c_address: u16,
    pub gyroscope_offset_z: i16,
    pub gyroscope_offset_y: i16,
    pub gyroscope_offset_x: i16,
    pub accelerometer_offset_z: i16,
    pub accelerometer_offset_y: i16,
    pub accelerometer_offset_x: i16,
}

#[derive(Debug, Clone, serde_derive::Deserialize, serde_derive::Serialize)]
pub struct RabbitMQ {
    pub hostname: String,
    pub port: u16,
    pub username: Option<String>,
    pub password: Option<String>,
    pub exchange: String,
}

impl Config {
    pub fn from_file(file: &str) -> Result<Config, Error> {
        let file_buffer: Vec<u8> = std::fs::read(file)?;
        let config: Config = toml::from_slice::<Config>(&file_buffer)?;

        Ok(config)
    }

    pub fn to_file(&self, file: &str) -> Result<(), Error> {
        let file_buffer: Vec<u8> = toml::to_vec(self)?;

        std::fs::write(file, &file_buffer)?;

        Ok(())
    }
}

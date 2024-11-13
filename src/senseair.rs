#![deny(missing_docs)]
#![no_std]


use byteorder::{BigEndian, ByteOrder};
use core::cell::RefCell;
use core::str;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal::digital::v2::{InputPin, OutputPin};

use heapless::String;
use heapless::Vec;

const EXPECT_MSG: &str = "Vec was not large enough";
const ADDRESS: u8 = 0x68;

pub enum Registers {
    ErrorStatus = 0x00,
    MeasuredFilteredPc = 0x06,
    Temperature = 0x08,
    MeasurementCount = 0x0D,
    MeasurCycleTime = 0x0E,
    MeasUnfPressCompens = 0x10,
    MeasFilPressCompens = 0x12,
    MeasuredUnfiltered = 0x14,
    FirmwareType = 0x2F,
    FirmwareVer = 0x38,
    SensorId = 0x3A,
    ProductCode = 0x70,
    CalibrationStatus = 0x81,
    CalibrationCommand = 0x82,
    CalibrationTarget = 0x84,
    MeasurementMode_EE = 0x95,
    MeasurementPeriod_EE = 0x96,
    NumberOfSamples_EE = 0x98,
    ABC_Period_EE = 0x9A,
    ABC_Target_EE = 0x9E,
    StaticIIRFilter_EE = 0xA1,
    MeterControl_EE = 0xA5,
    I2C_Address_EE = 0xA7,
    Nominator_EE = 0xA8,
    Denominator_EE = 0xAA,
    Scale_ABC_Target = 0xB0,
    StartMesurement = 0xC3,
    PressureValue = 0xDC,
    AbcTime = 0xC4,
    ClearErrorStatus = 0x9D,
}

#[derive(Debug)]
pub enum ErrorStatus<E> {
    I2c(E),
    LowInternalRegulatedVoltage,
    MeasurementTimeout,
    AbnormalSignalLevel,
    ScaleFactorError,
    FatalError,
    I2cError,
    AlgoritmError,
    CalibrationError,
    SelfDiagnosticsError,
    OutOfRange,
    MemoryError,
    NoMeasurementCompleted,
}
#[derive(Default)]
pub struct ProductType {
    FirmwareType: u8,
    MainRevision: u8,
    SubRevision: u8,
    SensorId: u32,
    ProductCode: String<11>,
}

impl core::fmt::Debug for ProductType {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        fmt.debug_struct("ProductType")
            .field("FirmwareType", &self.FirmwareType)
            .field("MainRevision", &self.MainRevision)
            .field("SubRevision", &self.SubRevision)
            .field("SensorId", &self.SensorId)
            .field("ProductCode", &self.ProductCode)
            .finish()
    }
}

 #[cfg(feature = "defmt")]
impl defmt::Format for ProductType {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "ProductType {{");
        defmt::write!(fmt, "FirmwareType: {=u8}, ", self.FirmwareType);
        defmt::write!(fmt, "MainRevision: {=u8}, ", self.MainRevision);
        defmt::write!(fmt, "SubRevision: {=u8}, ", self.SubRevision);
        defmt::write!(fmt, "SensorId: {=u32}, ", self.SensorId);
        defmt::write!(fmt, "ProductCode: {}, ", self.ProductCode.as_str());
        defmt::write!(fmt, " }}");
    }
}
/////////////////////////////////////////////////////////////////
#[derive(Default, Clone)]
pub struct Config {
    pub single_measurement_mode: u8,
    pub measurement_period: u16,
    pub number_of_samples: u16,
    pub abc_period: u16,
    pub abc_target: u16,
    pub iir_filter: u8,
    pub meter_control: u8,
    pub i2c_address: u8,
    pub nominator: u16,
    pub denominator: u16,
    pub scaled_abc_target: u16,
}

impl core::fmt::Debug for Config {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        fmt.debug_struct("Config")
            .field(
                "SingleMeasurementMode",
                &format_args!("{:x}", self.single_measurement_mode),
            )
            .field("MeasurementPeriod", &self.measurement_period)
            .field("NumberOfSamples", &self.number_of_samples)
            .field("ABCPeriod", &self.abc_period)
            .field("ABCTarget", &self.abc_target)
            .field("IIRFilter", &self.iir_filter)
            .field("MeterControl", &self.meter_control)
            .field("I2CAddres", &self.i2c_address)
            .field("Nominator", &self.nominator)
            .field("Denominator", &self.denominator)
            .field("ScaledABCTarget", &self.scaled_abc_target)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Config {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Config {{");
        defmt::write!(
            fmt,
            "SingleMeasurementMode: {=u8}, ",
            self.single_measurement_mode
        );
        defmt::write!(fmt, "MeasurementPeriod: {=u16}, ", self.measurement_period);
        defmt::write!(fmt, "NumberOfSamples: {=u16}, ", self.number_of_samples);
        defmt::write!(fmt, "ABCPeriod: {=u16}, ", self.abc_period);
        defmt::write!(fmt, "ABCTarget: {=u16}, ", self.abc_target);
        defmt::write!(fmt, "IIRFilter: {=u8}, ", self.iir_filter);
        defmt::write!(fmt, "MeterControl: {=u8}, ", self.meter_control);
        defmt::write!(fmt, "I2CAddress: {=u8}, ", self.i2c_address);
        defmt::write!(fmt, "Nominator: {=u16}, ", self.nominator);
        defmt::write!(fmt, "Denominator: {=u16}, ", self.denominator);
        defmt::write!(fmt, "ScaledABCTarget: {=u16}", self.scaled_abc_target);
        defmt::write!(fmt, " }}");
    }
}

impl Config {
    fn from_bytes(buf: &[u8; 25]) -> Self {
        let single_measurement_mode = buf[0];
        let measurement_period = u16::from_be_bytes([buf[1], buf[2]]);
        let number_of_samples = u16::from_be_bytes([buf[3], buf[4]]);
        let abc_period = u16::from_be_bytes([buf[5], buf[6]]);
        let abc_target = u16::from_be_bytes([buf[8], buf[9]]);
        let iir_filter = buf[11];
        let meter_control = buf[15];
        let i2c_address = buf[18];
        let nominator = u16::from_be_bytes([buf[19], buf[20]]);
        let denominator = u16::from_be_bytes([buf[21], buf[22]]);
        let scaled_abc_target = u16::from_be_bytes([buf[23], buf[24]]);

        Self {
            single_measurement_mode,
            measurement_period,
            number_of_samples,
            abc_period,
            abc_target,
            iir_filter,
            meter_control,
            i2c_address,
            nominator,
            denominator,
            scaled_abc_target,
        }
    }
}

#[derive(PartialEq, Eq, Clone)]
pub struct Measurement {
    MeasuredFilteredPressComp: i16,
    Temperature: i16,
    MeasurementCount: u8,
    MeasurementCycleTime: u16,
    MeasuredUnfilteredPressComp: i16,
    MeasuredFiltered: i16,
    MeasuredUnfiltered: i16,
    ScaledMeasured: i16,
    ETC: u32,
}
impl core::fmt::Debug for Measurement {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        fmt.debug_struct("Measurement")
            .field("MeasuredFilteredPressComp", &self.MeasuredFilteredPressComp)
            .field("Temperature", &self.Temperature)
            .field("MeasurementCount", &self.MeasurementCount)
            .field("MeasurementCycleTime", &self.MeasurementCycleTime)
            .field(
                "MeasuredUnfilteredPressComp",
                &self.MeasuredUnfilteredPressComp,
            )
            .field("MeasuredFiltered", &self.MeasuredFiltered)
            .field("MeasuredUnfiltered", &self.MeasuredUnfiltered)
            .field("ScaledMeasured", &self.ScaledMeasured)
            .field("ETC", &self.ETC)
            .finish()
    }
}

// #[cfg(feature = "defmt")]
impl defmt::Format for Measurement {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Measurement {{");
        defmt::write!(
            fmt,
            "MeasuredFilteredPressComp: {=i16}, ",
            self.MeasuredFilteredPressComp
        );
        defmt::write!(fmt, "Temperature: {=i16}, ", self.Temperature);
        defmt::write!(fmt, "MeasurementCount: {=u8}, ", self.MeasurementCount);
        defmt::write!(
            fmt,
            "MeasurementCycleTime: {=u16}, ",
            self.MeasurementCycleTime
        );
        defmt::write!(
            fmt,
            "MeasuredUnfilteredPressComp: {=i16}, ",
            self.MeasuredUnfilteredPressComp
        );
        defmt::write!(fmt, "MeasuredFiltered: {=i16}, ", self.MeasuredFiltered);
        defmt::write!(fmt, "MeasuredUnfiltered: {=i16}, ", self.MeasuredUnfiltered);
        defmt::write!(fmt, "ScaledMeasured: {=i16}, ", self.ScaledMeasured);
        defmt::write!(fmt, "ETC: {=u32}, ", self.ETC);

        defmt::write!(fmt, " }}");
    }
}
impl Measurement {
    fn from_bytes(buf: &[u8; 28]) -> Self {
        let MeasuredFilteredPressComp = i16::from_be_bytes([buf[6], buf[7]]);
        let Temperature = i16::from_be_bytes([buf[8], buf[9]]);
        let MeasurementCount = buf[13];
        let MeasurementCycleTime = u16::from_be_bytes([buf[14], buf[15]]);
        let MeasuredUnfilteredPressComp = i16::from_be_bytes([buf[16], buf[17]]);
        let MeasuredFiltered = i16::from_be_bytes([buf[18], buf[19]]);
        let MeasuredUnfiltered = i16::from_be_bytes([buf[20], buf[21]]);
        let ScaledMeasured = i16::from_be_bytes([buf[22], buf[23]]);
        let ETC = u32::from_be_bytes([buf[24], buf[25], buf[26], buf[27]]);

        Self {
            MeasuredFilteredPressComp,
            Temperature,
            MeasurementCount,
            MeasurementCycleTime,
            MeasuredUnfilteredPressComp,
            MeasuredFiltered,
            MeasuredUnfiltered,
            ScaledMeasured,
            ETC,
        }
    }
}
/////////////////////////////////////////////////////////////////
pub struct Sunrise<'a, I2C, D, EN, NRDY> {
    comm: I2C,
    delay: &'a mut D,
    en_pin: Option<EN>,
    n_rdy_pin: NRDY,
    address: u8,
    state_buf: [u8; 24],
    config: Config,
    product_type: ProductType,
}

impl<'a, I2C, E, D, EN, NRDY> Sunrise<'a, I2C, D, EN, NRDY>
where
    I2C: Read<Error = E> + Write<Error = E>,
    D: DelayMs<u32> + 'a,
    EN: OutputPin,
    NRDY: InputPin,
{
    /// Creates a new instance of the `Sunrise` struct.
    ///
    /// This function initializes the `Sunrise` sensor with the provided I2C communication
    /// interface, delay mechanism, enable pin, and ready pin. It sets up necessary fields,
    /// including the I2C address and default values for the sensor's configuration and product type.
    ///
    /// # Parameters
    ///
    /// - `i2c`: An instance of a type that implements the I2C communication trait.
    /// - `delay`: A mutable reference to a delay object for timing operations.
    /// - `en_pin`: An optional enable pin that controls power to the sensor.
    /// - `nrdy_pin`: The non-ready pin that indicates when the sensor is ready for communication.
    ///
    /// # Returns
    ///
    /// Returns a new `Sunrise` instance, properly initialized and ready for use.
    pub fn new(i2c:I2C, delay: &'a mut D, en_pin: Option<EN>, nrdy_pin: NRDY) -> Self {
        Sunrise {
            comm: i2c,
            delay: delay,
            en_pin: en_pin,
            n_rdy_pin: nrdy_pin,
            address: ADDRESS,
            state_buf: [0x00; 24],
            config: Config::default(),
            product_type: ProductType::default(),
        }
    }

    /// Reads the sensor's ID, firmware type, revision, and product code if the firmware revision is 4.08 or later.
    ///
    /// # Returns
    ///
    /// This function returns a `Result`:
    /// - `Ok(())` if the operation was successful.
    /// - An error of type `E` if any communication or processing errors occur.
    ///
    /// # Operation
    ///
    /// The function performs the following steps:
    /// 1. It initializes a buffer to store the data read from the sensor.
    /// 2. It sends commands to read the firmware type, firmware version, and sensor ID from the specified registers.
    /// 3. The firmware version is checked, and if it is 4.08 or later, it reads the product code as well.
    /// 4. The read data is stored in the `product_type` structure, which holds information about:
    ///    - Firmware type
    ///    - Main revision
    ///    - Sub revision
    ///    - Sensor ID
    ///    - Product code (if supported)
    ///
    fn product_type_get(&mut self) -> Result<(), E> {
        let mut buf = [0u8; 18];
        let mut vec: Vec<u8, 11> = Vec::new();

        self.comm
            .write(self.address, &(Registers::FirmwareType as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut [buf[0]])?;
        self.comm
            .write(self.address, &(Registers::FirmwareVer as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf[1..3])?;
        self.comm
            .write(self.address, &(Registers::SensorId as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf[3..7])?;

        let id = u32::from_be_bytes([buf[3], buf[4], buf[5], buf[6]]);

        self.product_type.FirmwareType = buf[0];
        self.product_type.MainRevision = buf[1];
        self.product_type.SubRevision = buf[2];
        self.product_type.SensorId = u32::from_be_bytes([buf[3], buf[4], buf[5], buf[6]]);

        if self.product_type.MainRevision >= 4 && self.product_type.SubRevision >= 8 {
            self.comm
                .write(self.address, &(Registers::ProductCode as u8).to_be_bytes())?;
            self.comm.read(self.address, &mut buf[7..])?;
            vec.extend_from_slice(&buf[7..]).expect(EXPECT_MSG);
            self.product_type.ProductCode = String::from_utf8(vec).unwrap();
        } else {
            self.product_type.ProductCode = String::try_from("No Supporte").unwrap();
        }

        Ok(())
    }

    /// Checks the sensor error register for specific error conditions.
    ///
    /// # Parameters
    ///
    /// - `sensor_err`: A 16-bit unsigned integer representing the error status bits returned by the sensor.
    ///
    /// # Operation
    ///
    /// This function examines the provided `sensor_err` value to determine if any specific error conditions are indicated.
    /// Each error condition corresponds to a specific bit in the `sensor_err` value:
    /// - Bit 16: Low Internal Regulated Voltage
    /// - Bit 15: Measurement Timeout
    /// - Bit 14: Abnormal Signal Level
    /// - Bit 8: Scale Factor Error
    /// - Bit 7: Fatal Error
    /// - Bit 6: I2C Error
    /// - Bit 5: Algorithm Error
    /// - Bit 4: Calibration Error
    /// - Bit 3: Self Diagnostics Error
    /// - Bit 2: Out Of Range
    /// - Bit 1: Memory Error
    /// - Bit 0: No Measurement Completed
    ///
    /// If any of these bits are set in `sensor_err`, the corresponding `ErrorStatus` is returned wrapped in an `Option`.
    /// If no errors are detected, `None` is returned.
    ///
    /// # Returns
    ///
    /// Returns an `Option<ErrorStatus<E>>`:
    /// - `Some(ErrorStatus)` if an error condition is detected, indicating the type of error.
    /// - `None` if no errors are present.
    ///
    fn check_sensor_error(&mut self, sensor_err: u16) -> Option<ErrorStatus<E>> {
        if sensor_err & (1 << 16) as u16 != 0 {
            Some(ErrorStatus::LowInternalRegulatedVoltage)
        } else if sensor_err & (1 << 15) as u16 != 0 {
            Some(ErrorStatus::MeasurementTimeout)
        } else if sensor_err & (1 << 14) as u16 != 0 {
            Some(ErrorStatus::AbnormalSignalLevel)
        } else if sensor_err & (1 << 8) as u16 != 0 {
            Some(ErrorStatus::ScaleFactorError)
        } else if sensor_err & (1 << 7) as u16 != 0 {
            Some(ErrorStatus::FatalError)
        } else if sensor_err & (1 << 6) as u16 != 0 {
            Some(ErrorStatus::I2cError)
        } else if sensor_err & (1 << 5) as u16 != 0 {
            Some(ErrorStatus::AlgoritmError)
        } else if sensor_err & (1 << 4) as u16 != 0 {
            Some(ErrorStatus::CalibrationError)
        } else if sensor_err & (1 << 3) as u16 != 0 {
            Some(ErrorStatus::SelfDiagnosticsError)
        } else if sensor_err & (1 << 2) as u16 != 0 {
            Some(ErrorStatus::OutOfRange)
        } else if sensor_err & (1 << 1) as u16 != 0 {
            Some(ErrorStatus::MemoryError)
        } else if sensor_err & (1 << 0) as u16 != 0 {
            Some(ErrorStatus::NoMeasurementCompleted)
        } else {
            None
        }
    }

    /// Resets the error status register of the sensor.
    ///
    /// # Returns
    ///
    /// Returns `Result<(), E>`, where:
    /// - `Ok(())` indicates that the error status register has been successfully reset.
    /// - `Err(E)` indicates an error occurred during the I2C communication process.
    ///
    /// # Usage
    ///
    /// Call this function when you need to reset any error status flags in the sensor, allowing for normal operation
    /// to continue without being affected by previous error conditions.
    ///
    fn clear_error_status(&mut self) -> Result<(), E> {
        let mut vec: Vec<u8, 2> = Vec::new();
        vec.extend_from_slice(&(Registers::ClearErrorStatus as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x00 as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;
        Ok(())
    }

    // Initiates a new measurement by sending the start measurement command and transferring the sensor state data
    /// from the previous measurement cycle.
    ///
    /// # Returns
    ///
    /// Returns `Result<(), E>`, where:
    /// - `Ok(())` indicates that the command was successfully sent, and the sensor is now set up for a new measurement cycle.
    /// - `Err(E)` represents any error that occurred during the I2C communication process.
    ///
    /// # Usage
    ///
    /// Call this function before initiating a new measurement cycle to ensure the sensor starts with the correct settings and
    /// continuity from the previous measurement cycle.
    ///
    fn sensor_state_data_set(&mut self) -> Result<(), E> {
        let mut vec: Vec<u8, 26> = Vec::new();

        vec.extend_from_slice(&(Registers::StartMesurement as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x01 as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        vec.extend_from_slice(&self.state_buf).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;

        Ok(())
    }

    /// Reads the sensor's state data from the specified register range (0xC4 - 0xDB) and stores it for use in the next measurement cycle.
    ///
    /// # Returns
    ///
    /// Returns `Result<(), E>`, where:
    /// - `Ok(())` indicates that the data was read successfully.
    /// - `Err(E)` represents an error encountered during the I2C communication process.
    ///
    /// # Usage
    ///
    /// This function is typically called before a measurement operation to ensure that the latest sensor state data is available.

    fn sensor_state_data_get(&mut self) -> Result<(), E> {
        self.comm
            .write(self.address, &(Registers::AbcTime as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut self.state_buf)
    }

    fn is_equal<F: PartialEq>(&mut self, a: F, b: F) -> bool {
        a == b
    }

    /// Drives the sensor's EN (Enable) pin high, activating the sensor.
    ///
    /// # Details
    ///
    /// - The `en_pin_set` function sets the EN pin to a high state, enabling or powering up the sensor.
    /// - After setting the EN pin high, it waits for a minimum delay of 35 milliseconds to allow the sensor time to start up and stabilize.
    ///   This delay is required for proper sensor initialization and should not be reduced, as doing so might result in incorrect behavior or initialization failure.
    /// - The function only performs these actions if `self.en_pin` is initialized; if `self.en_pin` is `None`, it does nothing.
    ///
    /// # Usage
    ///
    /// Use this function to enable the sensor before performing any initialization or reading operations. Pair it with `en_pin_reset` to control
    /// the sensor’s power state.

    fn en_pin_set(&mut self) {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok();
            /// Wait for minimum 35ms for sensor start-up and stabilisation
            self.delay.delay_ms(35);
        }
    }

    /// Drives the sensor's EN (Enable) pin low, effectively disabling the sensor.
    ///
    /// # Details
    ///
    /// - The `en_pin_reset` function sets the EN pin to a low state. This typically disables or powers down the sensor,
    ///   depending on the sensor’s design and how the EN pin is implemented.
    /// - This function only attempts to set the EN pin low if `self.en_pin` has been initialized. If `self.en_pin` is `None`,
    ///   no action is taken.
    ///
    /// # Usage
    ///
    /// Use this function when you need to disable the sensor, such as to conserve power or to reset its state. Pair with `en_pin_set`
    /// for toggling the sensor’s power state.
    fn en_pin_reset(&mut self) {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok();
        }
    }

    /// Reads all configuration (EE) registers from the sensor and returns the current settings as a [`Config`] object.
    ///
    /// # Returns
    ///
    /// Returns `Result<Config, E>`, where:
    /// - `Ok(Config)` contains the sensor's configuration data read from the registers.
    /// - `Err(E)` represents an error encountered during the I2C communication process.
    ///
    /// # Operation
    ///
    /// - The function begins by enabling the sensor using `en_pin_set`.
    /// - It then sends a read command to the sensor, targeting the starting address of the EE registers (`MeasurementMode_EE`).
    /// - After the write command, it reads a series of bytes into a buffer (`buf`), representing the configuration data.
    /// - The function constructs a `Config` object by deserializing the buffer using `Config::from_bytes` and stores this as the current configuration (`self.config`).
    /// - Finally, the sensor is disabled using `en_pin_reset`, and the new configuration is returned.
    ///
    /// # Notes
    ///
    /// - The function reads a fixed 25-byte sequence from the sensor registers, which is expected to match the size required by `Config::from_bytes`.
    /// - The `en_pin_set` and `en_pin_reset` calls ensure the sensor is active only for the duration of the read operation, optimizing power usage in cases where continuous sensor operation is not required.
    pub fn get_config(&mut self) -> Result<Config, E> {
        let mut buf = [0u8; 25];

        self.en_pin_set();

        self.comm.write(
            self.address,
            &(Registers::MeasurementMode_EE as u8).to_be_bytes(),
        )?;
        self.comm.read(self.address, &mut buf)?;

        let read_config = Config::from_bytes(&buf);
        self.config = read_config.clone();

        self.en_pin_reset();

        Ok(read_config)
    }

    /// Sets configuration values in the sensor’s EE (EEPROM) registers.
    /// Each register is written only if the provided configuration value differs from the current one.
    ///
    /// # Arguments
    ///
    /// - `config`: A [`Config`] object containing desired configuration values for the sensor.
    ///   Fields in `config` that match current register values are ignored to avoid redundant writes.
    ///
    /// # Returns
    ///
    /// Returns `Result<(), E>`, where `Ok(())` indicates that all configuration settings were applied successfully,
    /// and `Err(E)` represents an error encountered during communication with the sensor.
    ///
    /// # Operation
    ///
    /// - The function begins by initializing a small buffer to hold up to 3 bytes per transaction.
    /// - For each configurable parameter in `config`, it checks whether the new value differs from the current value using `self.is_equal`.
    /// - If a value differs, it writes the corresponding register address and new value to the sensor using `self.comm.write`.
    /// - After each write, the buffer is cleared to prepare for the next potential write.
    ///
    /// # Notes
    ///
    /// This function assumes that the `comm.write` method can handle various data lengths depending on the register type,
    /// and relies on the `EXPECT_MSG` constant for handling potential buffer overflow errors.
    fn set_config(&mut self, config: Config) -> Result<(), E> {
        let mut vec: Vec<u8, 3> = Vec::new();

        if let false = self.is_equal(
            config.single_measurement_mode,
            self.config.single_measurement_mode,
        ) {
            vec.extend_from_slice(&(Registers::MeterControl_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.single_measurement_mode as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec[0..2])?;
            vec.clear();
        }
        if let false = self.is_equal(config.measurement_period, self.config.measurement_period) {
            vec.extend_from_slice(&(Registers::MeasurementPeriod_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.measurement_period as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }
        if let false = self.is_equal(config.abc_period, self.config.abc_period) {
            vec.extend_from_slice(&(Registers::ABC_Period_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.abc_period as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }
        if let false = self.is_equal(config.abc_target, self.config.abc_target) {
            vec.extend_from_slice(&(Registers::ABC_Target_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.abc_target as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.denominator, self.config.denominator) {
            vec.extend_from_slice(&(Registers::Denominator_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.denominator as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.nominator, self.config.nominator) {
            vec.extend_from_slice(&(Registers::Nominator_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.nominator as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.number_of_samples, self.config.number_of_samples) {
            vec.extend_from_slice(&(Registers::NumberOfSamples_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.number_of_samples as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.scaled_abc_target, self.config.scaled_abc_target) {
            vec.extend_from_slice(&(Registers::Scale_ABC_Target as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.scaled_abc_target as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.i2c_address, self.config.i2c_address) {
            vec.extend_from_slice(&(Registers::I2C_Address_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.i2c_address as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.iir_filter, self.config.iir_filter) {
            vec.extend_from_slice(&(Registers::StaticIIRFilter_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.iir_filter as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        Ok(())
    }
    /// Executes a background calibration procedure for the sensor.
    ///
    /// # Returns
    ///
    /// Returns `Result<(), ErrorStatus<E>>`, where `Ok(())` signifies successful calibration,
    /// and `Err(ErrorStatus::CalibrationError)` indicates that the calibration did not complete successfully.
    ///
    /// # Operation
    ///
    /// - First, the function enables the sensor using `en_pin_set`.
    /// - Then, it writes the calibration status and command registers to initiate calibration.
    /// - If `SingleMeasurementMode` is enabled, the function sets sensor data and waits for `n_rdy_pin` to go low, signaling readiness.
    /// - It then checks the calibration status register to confirm successful calibration (`0x20`).
    /// - If calibration fails, the function returns `Err(ErrorStatus::CalibrationError)`.
    ///
    /// # Panics
    ///
    /// This function will panic if any internal communication call fails unexpectedly. The expected errors
    /// are handled and returned as `ErrorStatus` variants.
    ///
    /// # Notes
    ///
    /// This function operates in a blocking manner, especially when `SingleMeasurementMode` is enabled, as it waits for the `n_rdy_pin` to signal readiness.
    /// Ensure this is acceptable in contexts where non-blocking behavior is essential.
    pub fn background_calibration(&mut self) -> Result<(), ErrorStatus<E>> {
        let mut vec: Vec<u8, 2> = Vec::new();
        let mut buf = [0u8; 1];

        self.en_pin_set();

        vec.extend_from_slice(&(Registers::CalibrationStatus as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x00 as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        self.comm
            .write(self.address, &vec)
            .map_err(ErrorStatus::I2c)?;
        vec.clear();

        vec.extend_from_slice(&(Registers::CalibrationCommand as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x07C06 as u16).to_be_bytes())
            .expect(EXPECT_MSG);
        self.comm
            .write(self.address, &vec)
            .map_err(ErrorStatus::I2c)?;

        if self.config.single_measurement_mode == 0x01 {
            self.sensor_state_data_set().map_err(ErrorStatus::I2c)?;

            loop {
                if let Ok(true) = self.n_rdy_pin.is_low() {
                    break;
                }
            }
        }

        self.comm
            .write(
                self.address,
                &(Registers::CalibrationStatus as u8).to_be_bytes(),
            )
            .map_err(ErrorStatus::I2c)?;
        self.comm
            .read(self.address, &mut buf)
            .map_err(ErrorStatus::I2c)?;

        self.en_pin_reset();

        if let 0x20 = buf[0] {
            if self.config.single_measurement_mode == 0x01 {
                self.sensor_state_data_get().map_err(ErrorStatus::I2c)?;
            }
            return Ok(());
        } else {
            return Err(ErrorStatus::CalibrationError);
        }
    }

    pub fn target_calibration(&mut self, value: u16) -> Result<(), E> {
        self.en_pin_set();
        self.comm.write(
            self.address,
            &(Registers::CalibrationTarget as u8).to_be_bytes(),
        )?;
        self.comm
            .write(self.address, &(value as u16).to_be_bytes())?;
        self.en_pin_reset();

        Ok(())
    }

    /// Initializes the sensor with optional configuration and identifies the device type.
    ///
    /// # Arguments
    ///
    /// - `config_sensor`: An optional [`Config`] parameter that, if provided, configures the sensor settings.
    ///
    /// # Returns
    ///
    /// Returns `Result<(), E>`, where `Ok(())` indicates successful initialization, and `Err(E)` signifies an error during the process.
    ///
    /// # Panics
    ///
    /// This function may panic if any of the internal sensor control functions return an error.
    ///
    /// # Notes
    ///
    /// - The function begins by enabling the sensor, retrieving its current state, and identifying its type.
    /// - If a configuration is provided, it is applied to the sensor.
    /// - Finally, the function resets the enable pin, completing the initialization process.
    pub fn init(&mut self, config_sensor: Option<Config>) -> Result<(), E> {
        self.en_pin_set();
        ///set sensor state data
        self.sensor_state_data_get()?;
        ///
        self.product_type_get()?;
        ///if config is none
        if let Some(config) = config_sensor {
            self.set_config(config)?;
        }
        self.en_pin_reset();

        Ok(())
    }

    /// Retrieves information about the sensor's firmware and product type.
    ///
    /// This function provides access to the sensor's ID, firmware type,
    /// firmware revision, and product code if the firmware revision is 4.08
    /// or later. The returned reference points to the `ProductType` structure
    /// which holds the relevant information.
    ///
    /// # Returns
    ///
    /// A reference to `ProductType` that contains:
    /// - `FirmwareType`: The type of firmware currently used by the sensor.
    /// - `MainRevision`: The main version of the firmware.
    /// - `SubRevision`: The sub-version of the firmware.
    /// - `SensorId`: The unique identifier for the sensor.
    /// - `ProductCode`: The product code, if applicable.
    ///
    pub fn fw_info_get(&mut self) -> &ProductType {
        &self.product_type
    }
    /// Initiates a CO2 measurement and retrieves the result.
    ///
    /// This function triggers a CO2 measurement process and returns the measurement
    /// data. It optionally accepts a pressure value to be set before the measurement
    /// is taken. The function handles sensor state and error management throughout
    /// the measurement process.
    ///
    /// # Arguments
    ///
    /// - `pressure`: An optional pressure value (in hPa) to be sent to the sensor.
    ///   If provided, it must be a 16-bit unsigned integer.
    ///
    /// # Returns
    ///
    /// Returns a `Result` that contains either:
    /// - `Ok(Measurement)`: A structure containing the measurement data, parsed
    ///   from the sensor's response.
    /// - `Err(ErrorStatus<E>)`: An error status indicating a failure during the
    ///   measurement process, which could be due to various sensor errors or I2C
    ///   communication issues.
    ///
    /// # Notes
    ///
    /// - This function is applicable for both single measurement mode and continuous
    ///   measurement mode, allowing flexibility in usage depending on the application
    pub fn CO2_measurement_get(
        &mut self,
        pressure: Option<u16>,
    ) -> Result<Measurement, ErrorStatus<E>> {
        let mut buf = [0u8; 28];

        self.en_pin_set();

        self.clear_error_status().map_err(ErrorStatus::I2c)?;
        self.sensor_state_data_set().map_err(ErrorStatus::I2c)?;

        if let Some(value) = pressure {
            self.comm
                .write(
                    self.address,
                    &(Registers::PressureValue as u8).to_be_bytes(),
                )
                .map_err(ErrorStatus::I2c)?;
            self.comm
                .write(self.address, &(value as u16).to_be_bytes())
                .map_err(ErrorStatus::I2c)?;
        }

        loop {
            if let Ok(true) = self.n_rdy_pin.is_low() {
                break;
            }
        }

        self.comm
            .write(self.address, &(Registers::ErrorStatus as u8).to_be_bytes())
            .map_err(ErrorStatus::I2c)?;
        self.comm
            .read(self.address, &mut buf)
            .map_err(ErrorStatus::I2c)?;

        self.sensor_state_data_get().map_err(ErrorStatus::I2c)?;

        self.en_pin_reset();

        let sensor_err = u16::from_be_bytes([buf[0], buf[1]]);

        if let Some(err) = self.check_sensor_error(sensor_err) {
            return Err(err);
        }

        Ok(Measurement::from_bytes(&buf))
    }
}


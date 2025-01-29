#![deny(missing_docs)]
#![no_std]

use byteorder::{BigEndian, ByteOrder};
use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex}; 
use core::str;
use embedded_hal::blocking::delay::DelayMs;
use cortex_m::delay::Delay;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use embedded_hal::digital::v2::{InputPin, OutputPin};

use heapless::String;
use heapless::Vec;

const EXPECT_MSG: &str = "Vec was not large enough";
const ADDRESS: u8 = 0x68;

#[derive(Copy, Clone)]
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
    ProductCode: String<16>,
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

pub trait DelayProvider {
    fn delay_ms(&mut self, ms: u32);
}
/////////////////////////////////////////////////////////////////
pub struct Sunrise<'a, I2C,  EN, NRDY> {
    comm: I2C,
    delay: &'a mut dyn DelayProvider,
    en_pin: Option<EN>,
    n_rdy_pin: NRDY,
    address: u8,
    state_buf: [u8; 24],
    config: Config,
    product_type: ProductType,
}

impl<'a, D: DelayMs<u32>> DelayProvider for Mutex<RefCell<D>> {
    fn delay_ms(&mut self, ms: u32) {
        cortex_m::interrupt::free(|cs| {
            let mut delay = self.borrow(cs).borrow_mut();
            delay.delay_ms(ms);
        });
    }
}

impl DelayProvider for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_ms(ms);
    }
}

impl<'a, I2C, E, EN, NRDY> Sunrise<'a, I2C, EN, NRDY>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    EN: OutputPin,
    NRDY: InputPin,
{
    

    pub fn new(i2c: I2C, delay: &'a mut dyn DelayProvider, en_pin: Option<EN>, nrdy_pin: NRDY) -> Self {
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

    fn read_register(&mut self, register: Registers, buf: &mut [u8]) -> Result<(), E> {
        self.comm
            .write_read(self.address, &[register as u8], buf)
            .or_else(|_| self.comm.write_read(self.address, &[register as u8], buf))?;
        Ok(())
    }

    fn write_register(&mut self, register: Registers, data: &[u8]) -> Result<(), E> {
        // Создаем новый heapless::Vec с фиксированным размером (например, 32 байта)
        let mut buf: Vec<u8, 32> = Vec::new();

        // Пытаемся добавить адрес регистра
        buf.push(register as u8).expect(EXPECT_MSG);

        // Добавляем данные в буфер
        buf.extend_from_slice(data).expect(EXPECT_MSG);

        // Отправляем данные через I2C
        self.comm
            .write(self.address, &buf)
            .or_else(|_| self.comm.write(self.address, &buf))?;

        Ok(())
    }

    /// Retrieves the sensor's firmware type, revision, sensor ID, and product code.
    fn product_type_get(&mut self) -> Result<(), E> {
        let mut vec: Vec<u8, 16> = Vec::new();
        let mut buf = [0u8; 16];

        self.read_register(Registers::FirmwareType, &mut buf[..1])?;
        self.product_type.FirmwareType = buf[0];

        self.read_register(Registers::FirmwareVer, &mut buf[..2])?;
        self.product_type.MainRevision = buf[0];
        self.product_type.SubRevision = buf[1];

        self.read_register(Registers::SensorId, &mut buf[..4])?;
        self.product_type.SensorId = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);

        if self.product_type.MainRevision >= 4 && self.product_type.SubRevision >= 8 {
            vec.extend_from_slice(&mut buf).expect(EXPECT_MSG);
            self.product_type.ProductCode = String::from_utf8(vec).unwrap();
        } else {
            self.product_type.ProductCode = String::try_from("No Supporte").unwrap();
        }

        Ok(())
    }

    /// Checks the sensor's error status register and returns an error status if any error conditions are detected.
    fn check_sensor_error(&mut self, sensor_err: u16) -> Option<ErrorStatus<E>> {
        match sensor_err {
            x if x & (1 << 15) != 0 => Some(ErrorStatus::LowInternalRegulatedVoltage),
            x if x & (1 << 14) != 0 => Some(ErrorStatus::MeasurementTimeout),
            x if x & (1 << 13) != 0 => Some(ErrorStatus::AbnormalSignalLevel),
            x if x & (1 << 8) != 0 => Some(ErrorStatus::ScaleFactorError),
            x if x & (1 << 7) != 0 => Some(ErrorStatus::FatalError),
            x if x & (1 << 6) != 0 => Some(ErrorStatus::I2cError),
            x if x & (1 << 5) != 0 => Some(ErrorStatus::AlgoritmError),
            x if x & (1 << 4) != 0 => Some(ErrorStatus::CalibrationError),
            x if x & (1 << 3) != 0 => Some(ErrorStatus::SelfDiagnosticsError),
            x if x & (1 << 2) != 0 => Some(ErrorStatus::OutOfRange),
            x if x & (1 << 1) != 0 => Some(ErrorStatus::MemoryError),
            x if x & (1 << 0) != 0 => Some(ErrorStatus::NoMeasurementCompleted),
            _ => None,
        }
    }

    /// Clears the sensor's error status register.  
    fn clear_error_status(&mut self) -> Result<(), E> {
        self.write_register(Registers::ClearErrorStatus, &[0x00u8])
    }

    /// Sets the sensor's measurement mode to single measurement mode.
    fn sensor_state_data_set(&mut self) -> Result<(), E> {
        let state_buf = self.state_buf.clone();
        self.write_register(Registers::AbcTime, &state_buf)
    }

    /// Retrieves the sensor's state data from the sensor's registers.
    fn sensor_state_data_get(&mut self) -> Result<(), E> {
        let mut state_buf: [u8; 24] = [0; 24];

        self.read_register(Registers::AbcTime, &mut state_buf)?;
        self.state_buf.copy_from_slice(&state_buf);
        Ok(())
    }
    /// Compares two values and returns a boolean indicating whether they are equal.
    fn is_equal<F: PartialEq>(&mut self, a: F, b: F) -> bool {
        a == b
    }
    /// Sets the enable pin high to power on the sensor.
    fn en_pin_set(&mut self) {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok();
            /// Wait for minimum 35ms for sensor start-up and stabilisation
            self.delay.delay_ms(35);
        }
    }
    /// Resets the enable pin to power off the sensor.
    fn en_pin_reset(&mut self) {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok();
        }
    }
    /// Retrieves the sensor's configuration values from the sensor's EEPROM registers.
    pub fn get_config(&mut self) -> Result<Config, E> {
        let mut buf = [0u8; 2];

        self.read_register(
            Registers::MeasurementMode_EE,
            &mut [self.config.single_measurement_mode],
        )?;
        self.read_register(Registers::MeasurementPeriod_EE, &mut buf)?;
        self.config.measurement_period = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::NumberOfSamples_EE, &mut buf)?;
        self.config.number_of_samples = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::ABC_Period_EE, &mut buf)?;
        self.config.abc_period = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::ABC_Target_EE, &mut buf)?;
        self.config.abc_target = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::StaticIIRFilter_EE, &mut [self.config.iir_filter])?;
        self.read_register(Registers::MeterControl_EE, &mut [self.config.meter_control])?;
        self.read_register(Registers::I2C_Address_EE, &mut [self.config.i2c_address])?;
        self.read_register(Registers::Nominator_EE, &mut buf)?;
        self.config.nominator = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::Denominator_EE, &mut buf)?;
        self.config.denominator = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::Scale_ABC_Target, &mut buf)?;
        self.config.scaled_abc_target = u16::from_be_bytes([buf[0], buf[1]]);

        let read_config = self.config.clone();

        Ok(read_config)
    }

    /// Sets the sensor's configuration values in the sensor's EEPROM registers.
    fn set_config(&mut self, config: Config) -> Result<(), E> {
        let mut vec: Vec<u8, 3> = Vec::new();

        if let false = self.is_equal(
            config.single_measurement_mode,
            self.config.single_measurement_mode,
        ) {
            self.write_register(
                Registers::MeterControl_EE,
                &[config.single_measurement_mode],
            )?;
        }
        if let false = self.is_equal(config.measurement_period, self.config.measurement_period) {
            self.write_register(
                Registers::MeasurementPeriod_EE,
                &config.measurement_period.to_be_bytes(),
            )?;
        }
        if let false = self.is_equal(config.abc_period, self.config.abc_period) {
            self.write_register(Registers::ABC_Period_EE, &config.abc_period.to_be_bytes())?;
        }
        if let false = self.is_equal(config.abc_target, self.config.abc_target) {
            self.write_register(Registers::ABC_Target_EE, &config.abc_target.to_be_bytes())?;
        }

        if let false = self.is_equal(config.denominator, self.config.denominator) {
            self.write_register(Registers::Denominator_EE, &config.denominator.to_be_bytes())?;
        }

        if let false = self.is_equal(config.nominator, self.config.nominator) {
            self.write_register(Registers::Nominator_EE, &config.nominator.to_be_bytes())?;
        }

        if let false = self.is_equal(config.number_of_samples, self.config.number_of_samples) {
            self.write_register(
                Registers::NumberOfSamples_EE,
                &config.number_of_samples.to_be_bytes(),
            )?;
        }

        if let false = self.is_equal(config.scaled_abc_target, self.config.scaled_abc_target) {
            self.write_register(
                Registers::Scale_ABC_Target,
                &config.scaled_abc_target.to_be_bytes(),
            )?;
        }

        if let false = self.is_equal(config.i2c_address, self.config.i2c_address) {
            self.write_register(Registers::I2C_Address_EE, &[config.i2c_address])?;
        }

        if let false = self.is_equal(config.iir_filter, self.config.iir_filter) {
            self.write_register(Registers::StaticIIRFilter_EE, &[config.iir_filter])?;
        }

        Ok(())
    }

    
    pub fn background_calibration(&mut self) -> Result<(), ErrorStatus<E>> {

        let mut buf = [0u8; 1];

        self.write_register(Registers::CalibrationStatus, &[0x00])
            .map_err(ErrorStatus::I2c)?;


        self.write_register(Registers::CalibrationCommand, &[0x07, 0xC6])
            .map_err(ErrorStatus::I2c)?;

        if self.config.single_measurement_mode == 0x01 {
            self.sensor_state_data_set().map_err(ErrorStatus::I2c)?;

            loop {
                if let Ok(true) = self.n_rdy_pin.is_low() {
                    break;
                }
            }
        }

        self.read_register(Registers::CalibrationStatus, &mut buf)
            .map_err(ErrorStatus::I2c)?;

        if let 0x20 = buf[0] {
            if self.config.single_measurement_mode == 0x01 {
                self.sensor_state_data_get().map_err(ErrorStatus::I2c)?;
            }
            return Ok(());
        } else {
            return Err(ErrorStatus::CalibrationError);
        }
    }

    /// Sets the target calibration value for the sensor.
    pub fn target_calibration(&mut self, value: u16) -> Result<(), E> {

        self.write_register(Registers::CalibrationTarget, &value.to_be_bytes())?;

        Ok(())
    }

    /// Initiates a CO2 measurement and retrieves the result.
    pub fn init(&mut self, config_sensor: Option<Config>) -> Result<(), E> {

        ///set sensor state data
        self.sensor_state_data_get()?;
        ///
        self.product_type_get()?;
        self.get_config()?;
        ///if config is none
        if let Some(config) = config_sensor {
            self.set_config(config)?;
        }

        Ok(())
    }

    /// Retrieves a CO2 measurement from the sensor.
    pub fn CO2_measurement_get(
        &mut self,
        pressure: Option<u16>,
    ) -> Result<Measurement, ErrorStatus<E>> {
        let mut buf = [0u8; 28];

        self.clear_error_status().map_err(ErrorStatus::I2c)?;
        self.sensor_state_data_set().map_err(ErrorStatus::I2c)?;

        if let Some(value) = pressure {
            self.write_register(Registers::PressureValue, &value.to_be_bytes())
                .map_err(ErrorStatus::I2c)?;
        }

        loop {
            if let Ok(true) = self.n_rdy_pin.is_low() {
                break;
            }
        }

        // self.comm
        //     .write(self.address, &(Registers::ErrorStatus as u8).to_be_bytes())
        //     .map_err(ErrorStatus::I2c)?;
        self.read_register(Registers::ErrorStatus, &mut buf[..2])
            .map_err(ErrorStatus::I2c)?;

        self.sensor_state_data_get().map_err(ErrorStatus::I2c)?;


        let sensor_err = u16::from_be_bytes([buf[0], buf[1]]);

        if let Some(err) = self.check_sensor_error(sensor_err) {
            return Err(err);
        }

        Ok(Measurement::from_bytes(&buf))
    }
}

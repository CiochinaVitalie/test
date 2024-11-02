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
    ProductCode: String<20>,
}
/////////////////////////////////////////////////////////////////
#[derive(Default, Clone)]
pub struct Config {
    pub SingleMeasurementMode: u8,
    pub MeasurementPeriod: u16,
    pub NumberOfSamples: u16,
    pub ABCPeriod: u16,
    pub ABCTarget: u16,
    pub IIRFilter: u8,
    pub MeterControl: u8,
    pub I2CAddres: u8,
    pub Nominator: u16,
    pub Denominator: u16,
    pub ScaledABCTarget: u16,
}

impl core::fmt::Debug for Config {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        fmt.debug_struct("Config")
            .field(
                "SingleMeasurementMode",
                &format_args!("{:x}", self.SingleMeasurementMode),
            )
            .field("MeasurementPeriod", &self.MeasurementPeriod)
            .field("NumberOfSamples", &self.NumberOfSamples)
            .field("ABCPeriod", &self.ABCPeriod)
            .field("ABCTarget", &self.ABCTarget)
            .field("IIRFilter", &self.IIRFilter)
            .field("MeterControl", &self.MeterControl)
            .field("I2CAddres", &self.I2CAddres)
            .field("Nominator", &self.Nominator)
            .field("Denominator", &self.Denominator)
            .field("ScaledABCTarget", &self.ScaledABCTarget)
            .finish()
    }
}

// #[cfg(feature = "defmt")]
impl defmt::Format for Config {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Config {{");
        defmt::write!(
            fmt,
            "SingleMeasurementMode: {=u8}, ",
            self.SingleMeasurementMode
        );
        defmt::write!(fmt, "MeasurementPeriod: {=u16}, ", self.MeasurementPeriod);
        defmt::write!(fmt, "NumberOfSamples: {=u16}, ", self.NumberOfSamples);
        defmt::write!(fmt, "ABCPeriod: {=u16}, ", self.ABCPeriod);
        defmt::write!(fmt, "ABCTarget: {=u16}, ", self.ABCTarget);
        defmt::write!(fmt, "IIRFilter: {=u8}, ", self.IIRFilter);
        defmt::write!(fmt, "MeterControl: {=u8}, ", self.MeterControl);
        defmt::write!(fmt, "I2CAddres: {=u8}, ", self.I2CAddres);
        defmt::write!(fmt, "Nominator: {=u16}, ", self.Nominator);
        defmt::write!(fmt, "Denominator: {=u16}, ", self.Denominator);
        defmt::write!(fmt, "ScaledABCTarget: {=u16}", self.ScaledABCTarget);
        defmt::write!(fmt, " }}");
    }
}

impl Config {
    fn from_bytes(buf: &[u8; 25]) -> Self {
        let SingleMeasurementMode = u8::from(buf[0]);
        let MeasurementPeriod = (u16::from(buf[1]) << 8) | u16::from(buf[2]);
        let NumberOfSamples = (u16::from(buf[3]) << 8) | u16::from(buf[4]);
        let ABCPeriod = (u16::from(buf[5]) << 8) | u16::from(buf[6]);
        let ABCTarget = (u16::from(buf[8]) << 8) | u16::from(buf[9]);
        let IIRFilter = u8::from(buf[11]);
        let MeterControl = u8::from(buf[15]);
        let I2CAddres = u8::from(buf[18]);
        let Nominator = (u16::from(buf[19]) << 8) | u16::from(buf[20]);
        let Denominator = (u16::from(buf[21]) << 8) | u16::from(buf[22]);
        let ScaledABCTarget = (u16::from(buf[23]) << 8) | u16::from(buf[24]);

        Self {
            SingleMeasurementMode,
            MeasurementPeriod,
            NumberOfSamples,
            ABCPeriod,
            ABCTarget,
            IIRFilter,
            MeterControl,
            I2CAddres,
            Nominator,
            Denominator,
            ScaledABCTarget,
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone)]
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
pub struct Sunrise<'a, T, D, EN, NRDY> {
    comm: T,
    delay: &'a mut D,
    en_pin: Option<EN>,
    n_rdy_pin: NRDY,
    address: u8,
    state_buf: [u8; 24],
    config: Config,
    product_type: ProductType,
}

impl<'a, T, E, D, EN, NRDY> Sunrise<'a, T, D, EN, NRDY>
where
    T: Read<Error = E> + Write<Error = E>,
    D: DelayMs<u32> + 'a,
    EN: OutputPin,
    NRDY: InputPin,
{
    pub fn new(i2c: T, delay: &'a mut D, en_pin: Option<EN>, nrdy_pin: NRDY) -> Self {
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

    fn product_type_get(&mut self) -> Result<(), E> {
        let mut buf = [0u8; 18];
        let mut vec: Vec<u8, 20> = Vec::new();

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

        //
        Ok(())
    }

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

    fn clear_error_status(&mut self) -> Result<(), E> {
        let mut vec: Vec<u8, 2> = Vec::new();
        vec.extend_from_slice(&(Registers::ClearErrorStatus as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x00 as u8).to_be_bytes())
            .expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;
        Ok(())
    }

    fn sensor_state_data_get(&mut self) -> Result<(), E> {
        self.comm
            .write(self.address, &(Registers::AbcTime as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut self.state_buf)
    }

    fn is_equal<F: PartialEq>(&mut self, a: F, b: F) -> bool {
        a == b
    }

    fn en_pin_set(&mut self) {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok();
            self.delay.delay_ms(35);
        }
    }

    fn en_pin_reset(&mut self) {
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok();
        }
    }

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

    fn set_config(&mut self, config: Config) -> Result<(), E> {
        let mut vec: Vec<u8, 3> = Vec::new();

        if let false = self.is_equal(
            config.SingleMeasurementMode,
            self.config.SingleMeasurementMode,
        ) {
            vec.extend_from_slice(&(Registers::MeterControl_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.SingleMeasurementMode as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec[0..2])?;
            vec.clear();
        }
        if let false = self.is_equal(config.MeasurementPeriod, self.config.MeasurementPeriod) {
            vec.extend_from_slice(&(Registers::MeasurementPeriod_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.MeasurementPeriod as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }
        if let false = self.is_equal(config.ABCPeriod, self.config.ABCPeriod) {
            vec.extend_from_slice(&(Registers::ABC_Period_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.ABCPeriod as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }
        if let false = self.is_equal(config.ABCTarget, self.config.ABCTarget) {
            vec.extend_from_slice(&(Registers::ABC_Target_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.ABCTarget as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.Denominator, self.config.Denominator) {
            vec.extend_from_slice(&(Registers::Denominator_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.Denominator as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.Nominator, self.config.Nominator) {
            vec.extend_from_slice(&(Registers::Nominator_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.Nominator as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.NumberOfSamples, self.config.NumberOfSamples) {
            vec.extend_from_slice(&(Registers::NumberOfSamples_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.NumberOfSamples as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.ScaledABCTarget, self.config.ScaledABCTarget) {
            vec.extend_from_slice(&(Registers::Scale_ABC_Target as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.ScaledABCTarget as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.I2CAddres, self.config.I2CAddres) {
            vec.extend_from_slice(&(Registers::I2C_Address_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.I2CAddres as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.IIRFilter, self.config.IIRFilter) {
            vec.extend_from_slice(&(Registers::StaticIIRFilter_EE as u8).to_be_bytes())
                .expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.IIRFilter as u16).to_be_bytes())
                .expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        Ok(())
    }

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

        if self.config.SingleMeasurementMode == 0x01 {
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
            if self.config.SingleMeasurementMode == 0x01 {
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

    pub fn init(&mut self, config_sensor: Option<Config>) -> Result<(), E> {
        self.en_pin_set();

        self.sensor_state_data_get()?;
        self.product_type_get()?;

        if let Some(config) = config_sensor {
            self.set_config(config)?;
        }
        self.en_pin_reset();

        Ok(())
    }
    pub fn fw_info_get(&mut self) -> &ProductType {
        &self.product_type
    }

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
                .write(
                    self.address,
                    &(value as u16).to_be_bytes(),
                )
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

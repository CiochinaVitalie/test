#![deny(missing_docs)]
#![no_std]

use byteorder::{BigEndian, ByteOrder};
use rp_pico::pac::adc::cs::R;
use sx127x_lora::RadioMode;
use core::cell::RefCell;
use core::str;
use cortex_m::delay::Delay;
use cortex_m::interrupt::{self, Mutex};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use embedded_hal::digital::v2::{InputPin, OutputPin};

use heapless::String;
use heapless::Vec;

const EXPECT_MSG: &str = "Vec was not large enough";
const ADDRESS: u8 = 0x68;
const RAM_DELAY: u32 = 1;
const EEPROM_DELAY: u32 = 25;
const NO_DELAY: u32 = 0;

#[derive(Copy, Clone)]
#[allow(dead_code)]
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
    MeasurementModeEe = 0x95,
    MeasurementPeriodEe = 0x96,
    NumberOfSamplesEe = 0x98,
    AbcPeriodEe = 0x9A,
    AbcTargetEe = 0x9E,
    StaticIIRFilterEe = 0xA1,
    MeterControlEe = 0xA5,
    I2cAddressEe = 0xA7,
    NominatorEe = 0xA8,
    DenominatorEe = 0xAA,
    ScaleAbcTarget = 0xB0,
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
#[derive(Default, Debug)]
pub struct ProductType {
    firmware_type: u8,
    main_revision: u8,
    sub_revision: u8,
    sensor_id: u32,
    product_code: String<16>,
}


#[cfg(feature = "defmt")]
impl defmt::Format for ProductType {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "ProductType {{\n");
        defmt::write!(fmt, "FirmwareType: {=u8},\n ", self.firmware_type);
        defmt::write!(fmt, "MainRevision: {=u8}, \n", self.main_revision);
        defmt::write!(fmt, "SubRevision: {=u8}, \n", self.main_revision);
        defmt::write!(fmt, "SensorId: {=u32}, \n", self.sensor_id);
        defmt::write!(fmt, "ProductCode: {:?}, \n", self.product_code.as_str());
        defmt::write!(fmt, " \n}}");
    }
}
/////////////////////////////////////////////////////////////////
#[derive(Default, Clone, Debug, defmt::Format)]
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

#[derive(Debug,Default, defmt::Format)]
pub struct SetData {
    abc_time:u16,
    abc_par0:u16,
    abc_par1:u16,
    abc_par2:u16,
    abc_par3:u16,
    filter_par0:u16,
    filter_par1:u16,
    filter_par2:u16,
    filter_par3:u16,
    filter_par4:u16,
    filter_par5:u16,
    filter_par6:u16,
}

#[derive(Clone, Debug,Default, defmt::Format)]
pub struct Measurement {
    measured_filtered_press_comp: i16,
    temperature: i16,
    measurement_count: u8,
    measurement_cycle_time: u16,
    measured_unfiltered_press_comp: i16,
    measured_filtered: i16,
    measured_unfiltered: i16,
    scaled_measured: i16,
    etc: u32,
}

pub trait DelayProvider {
    fn delay_ms(&mut self, ms: u32);
}
/////////////////////////////////////////////////////////////////
pub struct Sunrise<'a, I2C, EN, NRDY> {
    comm: I2C,
    delay: &'a mut dyn DelayProvider,
    en_pin: Option<EN>,
    n_rdy_pin: NRDY,
    address: u8,
    state_buf: [u8; 24],
    config: Config,
    product_type: ProductType,
    mesurement: Measurement,
    set_data: SetData,
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
    pub fn new(
        i2c: I2C,
        delay: &'a mut dyn DelayProvider,
        en_pin: Option<EN>,
        nrdy_pin: NRDY,
    ) -> Self {
        Sunrise {
            comm: i2c,
            delay: delay,
            en_pin: en_pin,
            n_rdy_pin: nrdy_pin,
            address: ADDRESS,
            state_buf: [0x00; 24],
            config: Config::default(),
            product_type: ProductType::default(),
            mesurement: Measurement::default(),
            set_data: SetData::default(),
        }
    }
    pub fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    fn read_register(&mut self, register: Registers, buf: &mut [u8]) -> Result<(), E> {
        self.comm
            .write_read(self.address, &[register as u8], buf)
            .or_else(|_| self.comm.write_read(self.address, &[register as u8], buf))?;
        Ok(())
    }

    fn write_register(&mut self, register: Registers, data: &[u8],delay:u32) -> Result<(), E> {
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

        self.delay_ms(delay);

        Ok(())
    }

    /// Retrieves the sensor's firmware type, revision, sensor ID, and product code.
    fn product_type_get(&mut self) -> Result<(), E> {
        let mut vec: Vec<u8, 16> = Vec::new();
        let mut buf = [0u8; 16];

        self.read_register(Registers::FirmwareType, &mut buf[..1])?;
        self.product_type.firmware_type = buf[0];

        self.read_register(Registers::FirmwareVer, &mut buf[..2])?;
        self.product_type.main_revision = buf[0];
        self.product_type.sub_revision = buf[1];

        self.read_register(Registers::SensorId, &mut buf[..4])?;
        self.product_type.sensor_id = u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]);

        if self.product_type.main_revision >= 4 && self.product_type.sub_revision >= 8 {
            vec.extend_from_slice(&mut buf).expect(EXPECT_MSG);
            self.product_type.product_code = String::from_utf8(vec).unwrap();
        } else {
            self.product_type.product_code = String::try_from("No Supporte").unwrap();
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
        self.write_register(Registers::ClearErrorStatus, &[0x00u8],RAM_DELAY)
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

        self.read_register(Registers::MeasurementModeEe, &mut buf)?;
        self.config.single_measurement_mode = buf[0];
        self.read_register(Registers::MeasurementPeriodEe, &mut buf)?;
        self.config.measurement_period = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::NumberOfSamplesEe, &mut buf)?;
        self.config.number_of_samples = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::AbcPeriodEe, &mut buf)?;
        self.config.abc_period = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::AbcTargetEe, &mut buf)?;
        self.config.abc_target = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::StaticIIRFilterEe, &mut buf)?;
        self.config.iir_filter = buf[0];
        self.read_register(Registers::MeterControlEe, &mut buf)?;
        self.config.meter_control = buf[0];
        self.read_register(Registers::I2cAddressEe, &mut buf)?;
        self.config.i2c_address = buf[0];
        self.read_register(Registers::NominatorEe, &mut buf)?;
        self.config.nominator = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::DenominatorEe, &mut buf)?;
        self.config.denominator = u16::from_be_bytes([buf[0], buf[1]]);
        self.read_register(Registers::ScaleAbcTarget, &mut buf)?;
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
            self.write_register(Registers::MeterControlEe, &[config.single_measurement_mode],EEPROM_DELAY)?;
        }
        if let false = self.is_equal(config.measurement_period, self.config.measurement_period) {
            self.write_register(
                Registers::MeasurementPeriodEe,
                &config.measurement_period.to_be_bytes(), EEPROM_DELAY
            )?;
        }
        if let false = self.is_equal(config.abc_period, self.config.abc_period) {
            self.write_register(Registers::AbcPeriodEe, &config.abc_period.to_be_bytes(), EEPROM_DELAY)?;
        }
        if let false = self.is_equal(config.abc_target, self.config.abc_target) {
            self.write_register(Registers::AbcTargetEe, &config.abc_target.to_be_bytes(), EEPROM_DELAY)?;
        }

        if let false = self.is_equal(config.denominator, self.config.denominator) {
            self.write_register(Registers::DenominatorEe, &config.denominator.to_be_bytes(),EEPROM_DELAY)?;
        }

        if let false = self.is_equal(config.nominator, self.config.nominator) {
            self.write_register(Registers::NominatorEe, &config.nominator.to_be_bytes(),EEPROM_DELAY)?;
        }

        if let false = self.is_equal(config.number_of_samples, self.config.number_of_samples) {
            self.write_register(
                Registers::NumberOfSamplesEe,
                &config.number_of_samples.to_be_bytes(),EEPROM_DELAY
            )?;
        }

        if let false = self.is_equal(config.scaled_abc_target, self.config.scaled_abc_target) {
            self.write_register(
                Registers::ScaleAbcTarget,
                &config.scaled_abc_target.to_be_bytes(),EEPROM_DELAY
            )?;
        }

        if let false = self.is_equal(config.i2c_address, self.config.i2c_address) {
            self.write_register(Registers::I2cAddressEe, &[config.i2c_address],EEPROM_DELAY)?;
        }

        if let false = self.is_equal(config.iir_filter, self.config.iir_filter) {
            self.write_register(Registers::StaticIIRFilterEe, &[config.iir_filter],EEPROM_DELAY)?;
        }

        Ok(())
    }

    pub fn background_calibration(&mut self) -> Result<(), ErrorStatus<E>> {
        let mut buf = [0u8; 1];

        self.write_register(Registers::CalibrationStatus, &[0x00],RAM_DELAY)
            .map_err(ErrorStatus::I2c)?;

        self.write_register(Registers::CalibrationCommand, &[0x07, 0xC6],RAM_DELAY)
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
        self.write_register(Registers::CalibrationTarget, &value.to_be_bytes(), RAM_DELAY)?;

        Ok(())
    }

    /// Initiates a CO2 measurement and retrieves the result.
    pub fn init(&mut self, config_sensor: Option<Config>) -> Result<(), E> {
        ///set sensor state data
        self.sensor_state_data_get()?;
        ///
        // self.product_type_get()?;
        // self.get_config()?;
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
    ) -> Result<&Measurement, ErrorStatus<E>> {
        let mut buf = [0u8; 2];

        self.clear_error_status().map_err(ErrorStatus::I2c)?;
        self.sensor_state_data_set().map_err(ErrorStatus::I2c)?;

        if let Some(value) = pressure {
            self.write_register(Registers::PressureValue, &value.to_be_bytes(),RAM_DELAY)
                .map_err(ErrorStatus::I2c)?;
        }

        // loop {
        //     if let Ok(true) = self.n_rdy_pin.is_low() {
        //         break;
        //     }
        // }
        self.delay.delay_ms(2400);

        let sensor_err = u16::from_be_bytes([buf[0], buf[1]]);

        if let Some(err) = self.check_sensor_error(sensor_err) {
            return Err(err);
        }

        self.read_register(Registers::ErrorStatus, &mut buf[..2])
            .map_err(ErrorStatus::I2c)?;

        self.read_register(Registers::MeasuredFilteredPc, &mut buf).map_err(ErrorStatus::I2c)?;
        self.mesurement.measured_filtered_press_comp = i16::from_be_bytes([buf[0], buf[1]]);

        self.read_register(Registers::Temperature, &mut buf).map_err(ErrorStatus::I2c)?;
        self.mesurement.temperature = i16::from_be_bytes([buf[0], buf[1]]);

        self.read_register(Registers::MeasurementCount, &mut buf)
            .map_err(ErrorStatus::I2c)?;
        self.mesurement.measurement_count = buf[0];

        self.read_register(Registers::MeasurCycleTime, &mut buf)
            .map_err(ErrorStatus::I2c)?;

        self.mesurement.measurement_cycle_time = u16::from_be_bytes([buf[0], buf[1]]);

        self.read_register(Registers::MeasUnfPressCompens, &mut buf)
            .map_err(ErrorStatus::I2c)?;

        self.mesurement.measured_unfiltered_press_comp = i16::from_be_bytes([buf[0], buf[1]]);

        self.read_register(Registers::MeasFilPressCompens, &mut buf)
            .map_err(ErrorStatus::I2c)?;

        self.mesurement.measured_filtered = i16::from_be_bytes([buf[0], buf[1]]);

        self.read_register(Registers::MeasuredUnfiltered, &mut buf)
            .map_err(ErrorStatus::I2c)?;

        self.mesurement.measured_unfiltered = i16::from_be_bytes([buf[0], buf[1]]); 


        self.sensor_state_data_get().map_err(ErrorStatus::I2c)?;


        Ok(&self.mesurement)
    }
}

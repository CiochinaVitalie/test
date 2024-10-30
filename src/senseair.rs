
use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal::digital::v2::{OutputPin,InputPin};
use embedded_hal::blocking::delay::DelayMs;
use byteorder::{BigEndian, ByteOrder};
use core::cell::RefCell;

use heapless::Vec;

const EXPECT_MSG: &str = "Vec was not large enough";
const ADDRESS: u8 = 0x68;


pub enum  Registers{
    ErrorStatus             = 0x00,
    MeasuredFilteredPc      = 0x06,
    Temperature             = 0x08,
    MeasurementCount        = 0x0D,
    MeasurCycleTime         = 0x0E,
    MeasUnfPressCompens     = 0x10,
    MeasFilPressCompens     = 0x12,
    MeasuredUnfiltered      = 0x14,
    FirmwareType            = 0x2F,
    FirmwareVer             = 0x38,
    SensoeId                = 0x3A,
    ProductCode             = 0x70,
    CalibrationStatus       = 0x81,
    CalibrationCommand      = 0x82,
    MeasurementMode_EE      = 0x95,
    MeasurementPeriod_EE    = 0x96,
    NumberOfSamples_EE      = 0x98,
    ABC_Period_EE           = 0x9A,
    ABC_Target_EE           = 0x9E,
    StaticIIRFilter_EE      = 0xA1,
    MeterControl_EE         = 0xA5,
    I2C_Address_EE          = 0xA7,
    Nominator_EE            = 0xA8,
    Denominator_EE          = 0xAA,
    Scale_ABC_Target        = 0xB0,
    StartMesurement         = 0xC3,
    AbcTime                 = 0xC4,
    ClearErrorStatus        = 0x9D
}

// #[derive(Debug)]
pub enum ErrorStatus <E>{
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
/////////////////////////////////////////////////////////////////
#[derive(Debug,Default,Clone)]
pub struct Config {
    pub SingleMeasurementMode: u8,
    pub MeasurementPeriod: u16,
    pub NumberOfSamples: u16,
    pub ABCPeriod: u16,
    pub ABCTarget:u16,
    pub IIRFilter:u8,
    pub MeterControl:u8,
    pub I2CAddres:u8,
    pub Nominator:u16,
    pub Denominator:u16,
    pub ScaledABCTarget:u16

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
            ScaledABCTarget
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct Measurement {
    pub co2:         u16,
    pub temperature: u16,
}

impl Measurement {
    fn from_bytes(buf: &[u8; 10]) -> Self {
        let co2 = (u16::from(buf[6]) << 8) | u16::from(buf[7]);
        let temperature = (u16::from(buf[8]) << 8) | u16::from(buf[9]);
        Self {
            co2,
            temperature,
        }
    }
}
/////////////////////////////////////////////////////////////////
pub struct Sunrise<'a,T,D,EN,NRDY> {
    comm: T,
    delay: &'a mut D,
    en_pin:Option<EN>,
    n_rdy_pin:NRDY,
    address: u8,
    state_buf:[u8;24],
    config:Config
}


impl<'a,T, E, D,EN,NRDY> Sunrise<'a,T,D,EN,NRDY> where T: Read<Error = E> + Write<Error = E>, D:DelayMs<u32> + 'a,EN:OutputPin,NRDY:InputPin {
    
    pub fn new_with_address(i2c: T, address: u8,delay:&'a mut D,en_pin:Option<EN>,nrdy_pin:NRDY) -> Self {
        Sunrise {
            comm: i2c,
            delay:delay,
            en_pin:en_pin,
            n_rdy_pin:nrdy_pin,
            address,
            state_buf:[0x00;24],
            config:Config::default()
        }
    }

    pub fn new(i2c: T, delay:&'a mut D,en_pin:Option<EN>,nrdy_pin:NRDY) -> Self {
        Sunrise {
            comm: i2c,
            delay:delay,
            en_pin:en_pin,
            n_rdy_pin:nrdy_pin,
            address: ADDRESS,
            state_buf:[0x00;24],
            config:Config::default()
        }
    }

    pub fn fimware_get(&mut self) -> Result<u16, E> {
        let mut buf = [0u8; 2];

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok();
            self.delay.delay_ms(35);
        }
           
        self.comm.write(self.address, &(Registers::FirmwareVer as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf)?;

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok();
        }
        Ok(u16::from_be_bytes(buf))
    }

    fn check_sensor_error(&mut self,sensor_err: u16) -> Option<ErrorStatus<E>> {
        if sensor_err & (1 << 16) != 0 {
            Some(ErrorStatus::LowInternalRegulatedVoltage)
        } else if sensor_err & (1 << 15) != 0 {
            Some(ErrorStatus::MeasurementTimeout)
        } else if sensor_err & (1 << 14) != 0 {
            Some(ErrorStatus::AbnormalSignalLevel)
        } else if sensor_err & (1 << 8) != 0 {
            Some(ErrorStatus::ScaleFactorError)
        } else if sensor_err & (1 << 7) != 0 {
            Some(ErrorStatus::FatalError)
        } else if sensor_err & (1 << 6) != 0 {
            Some(ErrorStatus::I2cError)
        } else if sensor_err & (1 << 5) != 0 {
            Some(ErrorStatus::AlgoritmError)
        } else if sensor_err & (1 << 4) != 0 {
            Some(ErrorStatus::CalibrationError)
        } else if sensor_err & (1 << 3) != 0 {
            Some(ErrorStatus::SelfDiagnosticsError)
        } else if sensor_err & (1 << 2) != 0 {
            Some(ErrorStatus::OutOfRange)
        } else if sensor_err & (1 << 1) != 0 {
            Some(ErrorStatus::MemoryError)
        } else if sensor_err & (1 << 0) != 0 {
            Some(ErrorStatus::NoMeasurementCompleted)
        }        
        else {
            None
        }
    }

    fn sensor_state_data_set(&mut self) -> Result<(), E> {

        let mut vec: Vec<u8, 26> = Vec::new();

        vec.extend_from_slice(&(Registers::StartMesurement as u8).to_be_bytes()).expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x01 as u8).to_be_bytes()).expect(EXPECT_MSG);
        vec.extend_from_slice(&self.state_buf).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;

        Ok(())
    }

    fn clear_error_status(&mut self) -> Result<(), E> {
        let mut vec: Vec<u8, 2> = Vec::new();
        vec.extend_from_slice(&(Registers::ClearErrorStatus as u8).to_be_bytes()).expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x00 as u8).to_be_bytes()).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;
        Ok(())
    }

    fn sensor_state_data_get(&mut self) -> Result<(), E> {

        self.comm.write(self.address, &(Registers::AbcTime as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut self.state_buf)
    }

    fn is_equal<F: PartialEq>(&mut self,a: F, b: F) -> bool {
        a == b
    }

    fn en_pin_set(&mut self){

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok();
        }
    }

    fn en_pin_reset(&mut self){

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok(); 
        }
    }

    pub fn get_config(&mut self)-> Result<Config, E> {
        let mut buf = [0u8; 25];

        self.en_pin_set();
        self.delay.delay_ms(35);

        self.comm.write(self.address, &(Registers::MeasurementMode_EE as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf)?;

        let read_config = Config::from_bytes(&buf);
        self.config = read_config.clone();

        self.en_pin_reset();
   
        Ok(read_config)

    }


   pub fn set_config(&mut self,config:Config) -> Result<(), E> {

        let mut vec: Vec<u8, 2> = Vec::new();

        self.en_pin_set();
        self.delay.delay_ms(35);
        
        if let false = self.is_equal(config.SingleMeasurementMode,self.config.SingleMeasurementMode)
        {
            vec.extend_from_slice(&(Registers::MeterControl_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.SingleMeasurementMode as u8).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }
        if let false = self.is_equal(config.MeasurementPeriod,self.config.MeasurementPeriod)
        {
            vec.extend_from_slice(&(Registers::MeasurementPeriod_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.MeasurementPeriod as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }
        if let false = self.is_equal(config.ABCPeriod,self.config.ABCPeriod)
        {
            vec.extend_from_slice(&(Registers::ABC_Period_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.ABCPeriod as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }
        if let false = self.is_equal(config.ABCTarget,self.config.ABCTarget)
        {
            vec.extend_from_slice(&(Registers::ABC_Target_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.ABCTarget as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.Denominator,self.config.Denominator)
        {
            vec.extend_from_slice(&(Registers::Denominator_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.Denominator as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.Nominator,self.config.Nominator)
        {
            vec.extend_from_slice(&(Registers::Nominator_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.Nominator as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.NumberOfSamples,self.config.NumberOfSamples)
        {
            vec.extend_from_slice(&(Registers::NumberOfSamples_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.NumberOfSamples as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.ScaledABCTarget,self.config.ScaledABCTarget)
        {
            vec.extend_from_slice(&(Registers::Scale_ABC_Target as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.ScaledABCTarget as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.I2CAddres,self.config.I2CAddres)
        {
            vec.extend_from_slice(&(Registers::I2C_Address_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.I2CAddres as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        if let false = self.is_equal(config.IIRFilter,self.config.IIRFilter)
        {
            vec.extend_from_slice(&(Registers::StaticIIRFilter_EE as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&(config.IIRFilter as u16).to_be_bytes()).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            vec.clear();
        }

        self.en_pin_reset();

     Ok(())
    }

    fn background_calibration(&mut self) -> Result<(), ErrorStatus<E>> {

        let mut vec: Vec<u8, 2> = Vec::new();
        let mut buf = [0u8; 1];
        vec.extend_from_slice(&(Registers::CalibrationStatus as u8).to_be_bytes()).expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x00 as u8).to_be_bytes()).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec).map_err(ErrorStatus::I2c)?;
        vec.clear();

        vec.extend_from_slice(&(Registers::CalibrationCommand as u8).to_be_bytes()).expect(EXPECT_MSG);
        vec.extend_from_slice(&(0x07C06 as u16).to_be_bytes()).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec).map_err(ErrorStatus::I2c)?;

        self.comm.write(self.address, &(Registers::CalibrationStatus as u8).to_be_bytes()).map_err(ErrorStatus::I2c)?;
        self.comm.read(self.address, &mut buf).map_err(ErrorStatus::I2c)?;

        if let 0x20 = buf[0]{
            return Ok(());
        }else{
            return Err(ErrorStatus::CalibrationError);
        }
    }

    pub fn init (&mut self,config:Config) -> Result<(), E> {

        self.en_pin_set();
        self.delay.delay_ms(35);
        
        self.sensor_state_data_get()?;
        self.set_config(config);

        self.en_pin_reset();

        Ok(())
    }

    pub fn CO2_measurement_get(&mut self) -> Result<Measurement, ErrorStatus<E>> {
 
        let mut buf = [0u8; 10];

        self.en_pin_set();
        self.delay.delay_ms(35);

        self.clear_error_status().map_err(ErrorStatus::I2c)?;
        self.sensor_state_data_set().map_err(ErrorStatus::I2c)?;

        loop
        {
            if let Ok(true) = self.n_rdy_pin.is_low()
            {              
               break; 
            }
        }

        self.comm.write(self.address, &(Registers::ErrorStatus as u8).to_be_bytes()).map_err(ErrorStatus::I2c)?;
        self.comm.read(self.address, &mut buf).map_err(ErrorStatus::I2c)?;

        self.sensor_state_data_get().map_err(ErrorStatus::I2c)?;


        self.en_pin_reset();

        let sensor_err = ((buf[0] as u16) << 8) | (buf[1] as u16);

        if let Some(err) = self.check_sensor_error(sensor_err) {
            return Err(err);
        }
 

        Ok(Measurement::from_bytes(&buf))
     
        
    }

}
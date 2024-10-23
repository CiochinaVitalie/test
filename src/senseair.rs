
use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal::digital::v2::{OutputPin,InputPin};
use embedded_hal::blocking::delay::DelayMs;
use byteorder::{BigEndian, ByteOrder};
use core::cell::RefCell;

use heapless::Vec;

const EXPECT_MSG: &str = "Vec was not large enough";
const ADDRESS: u8 = 0x68;
static mut STATE_DATA: [u8; 24] = [0x00;24];

pub enum ErrorStatus{
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

    MeasurementMode         = 0x95,
    MeterControl            = 0xA5,
    StartMesurement         = 0xC3,
    AbcTime                 = 0xC4,
    ClearErrorStatus        = 0x9D
}

/////////////////////////////////////////////////////////////////
#[derive(Debug)]
pub struct Measurement {
    pub co2:         u16,
    pub temperature: u16,
}

pub struct Sunrise<'a,T,D,EN,NRDY> {
    comm: T,
    delay: &'a mut D,
    en_pin:Option<EN>,
    n_rdy_pin:NRDY,
    address: u8,
    state_buf:[u8;24]
}

pub  struct My<T> {
    gh:Option<T>
}

impl<'a,T, E, D,EN,NRDY> Sunrise<'a,T,D,EN,NRDY> where T: Read<Error = E> + Write<Error = E>, D:DelayMs<u32> + 'a,EN:OutputPin,NRDY:InputPin {
    
    pub fn new_with_address(i2c: T, address: u8,delay:&'a mut D,en_pin:Option<EN>,nrdy_pin:NRDY) -> Self {
        Sunrise {
            comm: i2c,
            delay:delay,
            en_pin:en_pin,
            n_rdy_pin:nrdy_pin,
            address,
            state_buf:[0x00;24]
        }
    }

    pub fn new(i2c: T, delay:&'a mut D,en_pin:Option<EN>,nrdy_pin:NRDY) -> Self {
        Sunrise {
            comm: i2c,
            delay:delay,
            en_pin:en_pin,
            n_rdy_pin:nrdy_pin,
            address: ADDRESS,
            state_buf:[0x00;24]
        }
    }

    pub fn fimware_get(&mut self) -> Result<u16, E> {
        let mut buf = [0u8; 2];

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok(); 
        }
    
        self.delay.delay_ms(35);
        self.comm.write(self.address, &(Registers::FirmwareVer as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf)?;

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok();
        }
        Ok(u16::from_be_bytes(buf))
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

    pub fn init (&mut self,press_comp:bool, iir_filter:bool, abc:bool, continuous_mode:bool) -> Result<(), E> {

        let mut ctr_reg:u8 = 0;
        let mut vec: Vec<u8, 2> = Vec::new();

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok(); 
        }

        self.delay.delay_ms(35);
        self.sensor_state_data_get()?;

        self.comm.write(self.address, &(Registers::MeterControl as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut (ctr_reg).to_be_bytes())?;
        vec.extend_from_slice(&(Registers::MeterControl as u8).to_be_bytes()).expect(EXPECT_MSG);

        ctr_reg = if iir_filter {
            ctr_reg & 0xF3
        } else {
            ctr_reg | 0xFC
        };
        
        ctr_reg = if press_comp {
            ctr_reg & 0xEF
        } else {
            ctr_reg | 0xF0
        };

        ctr_reg = if abc {
            ctr_reg & 0xFD
        } else {
            ctr_reg |  0x02
        };

        vec.extend_from_slice(&(ctr_reg).to_be_bytes()).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;
        vec.clear();

        vec.extend_from_slice(&(Registers::MeasurementMode as u8).to_be_bytes()).expect(EXPECT_MSG);

        if continuous_mode{           
            vec.extend_from_slice(&(0x00 as u8).to_be_bytes()).expect(EXPECT_MSG);
            
        }
        else {
            vec.extend_from_slice(&(0x01 as u8).to_be_bytes()).expect(EXPECT_MSG);
        }

        self.comm.write(self.address, &vec)?;

        self.sensor_state_data_get()?;

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok(); 
        }
        Ok(())
    }

    pub fn enable_abc(&mut self)-> Result<(), E> {

        let mut ctr_reg:u8 = 0xFF;
        let mut vec: Vec<u8, 2> = Vec::new();

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok(); 
        }

        self.delay.delay_ms(35);
        self.comm.write(self.address, &(Registers::MeterControl as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut (ctr_reg).to_be_bytes())?;
        ctr_reg = ctr_reg & 0xFD;
        vec.extend_from_slice(&(ctr_reg).to_be_bytes()).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok(); 
        }
        Ok(())
    }
    pub fn disable_abc(&mut self)-> Result<(), E> {

        let mut ctr_reg:u8 = 0xFD;
        let mut vec: Vec<u8, 2> = Vec::new();
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok(); 
        }
        self.delay.delay_ms(35);
        self.comm.write(self.address, &(Registers::MeterControl as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut (ctr_reg).to_be_bytes())?;
        ctr_reg = ctr_reg & 02;
        vec.extend_from_slice(&(ctr_reg).to_be_bytes()).expect(EXPECT_MSG);
        self.comm.write(self.address, &vec)?;

        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok(); 
        }
        Ok(())
    }

    pub fn CO2_measurement_get(&mut self) -> Result<[u8; 8], E> {
        let mut vec: Vec<u8, 2> = Vec::new();
        let mut buf = [0u8; 8];
        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_high().ok(); 
        }
        self.delay.delay_ms(35);

        
        self.clear_error_status()?;
        self.sensor_state_data_set()?;

        loop
        {
            if let Ok(true) = self.n_rdy_pin.is_low()
            {              
               break; 
            }
        }

        self.comm.write(self.address, &(Registers::ErrorStatus as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf)?;

        self.sensor_state_data_get()?;


        if let Some(ref mut en_pin) = self.en_pin {
            en_pin.set_low().ok(); 
        }
        Ok(buf)
     
        
    }

}
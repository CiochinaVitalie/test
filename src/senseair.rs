
use cortex_m::delay;
use embedded_hal::blocking::i2c::{Read, Write};
use embedded_hal::digital::v2::{OutputPin,InputPin};
use embedded_hal::blocking::delay::DelayMs;
use byteorder::{BigEndian, ByteOrder};


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

    MeasurementMode         = 0x95,
    MeterControl            = 0xA5,
    StartMesurement         = 0xC3,
    AbcTime                 = 0xC4
}

/////////////////////////////////////////////////////////////////
#[derive(Debug)]
pub struct Measurement {
    pub co2:         u16,
    pub temperature: u16,
}

pub struct Sunrise<T,D,EN,NRDY> {
    comm: T,
    delay: D,
    en_pin:EN,
    n_rdy_pin:NRDY,
    address: u8,
}

impl<T, E, D,EN,NRDY> Sunrise<T,D,EN,NRDY> where T: Read<Error = E> + Write<Error = E>, D:DelayMs<u32>,EN:OutputPin,NRDY:InputPin {
    
    pub fn new_with_address(i2c: T, address: u8,delay:D,en_pin:EN,nrdy_pin:NRDY) -> Self {
        Sunrise {
            comm: i2c,
            delay:delay,
            en_pin:en_pin,
            n_rdy_pin:nrdy_pin,
            address
        }
    }

    pub fn new(i2c: T, delay:D,en_pin:EN,nrdy_pin:NRDY) -> Self {
        Sunrise {
            comm: i2c,
            delay:delay,
            en_pin:en_pin,
            n_rdy_pin:nrdy_pin,
            address: ADDRESS
        }
    }

    pub fn fimware_get(&mut self) -> Result<u16, E> {
        let mut buf = [0u8; 2];
        let _ = self.en_pin.set_high();
        self.delay.delay_ms(35);
        self.comm.write(self.address, &(Registers::FirmwareVer as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf)?;
        let _ = self.en_pin.set_low();
        Ok(u16::from_be_bytes(buf))
    }

     fn sensor_state_data_get(&mut self) -> Result<[u8; 24], E> {
        let mut buf = [0u8; 24];
       
        self.comm.write(self.address, &(Registers::StartMesurement as u8).to_be_bytes())?;
        self.comm.read(self.address, &mut buf)?;
        
        Ok(buf)
    }

    pub fn single_measurement_get(&mut self) -> Result<[u8; 8], E> {
        let mut vec: Vec<u8, 25> = Vec::new();
        let mut buf = [0u8; 8];
        let _ = self.en_pin.set_high();
        self.delay.delay_ms(35);

        if let Ok(state_dat) = self.sensor_state_data_get()
        {
            vec.extend_from_slice(&(Registers::MeasurementMode as u8).to_be_bytes()).expect(EXPECT_MSG);
            vec.extend_from_slice(&state_dat).expect(EXPECT_MSG);
            self.comm.write(self.address, &vec)?;
            
        }
        self.delay.delay_ms(2500);

        if let Ok(true) = self.n_rdy_pin.is_low()
        {
            self.comm.read(self.address, &mut buf)?;

        }else {
            
        }
        let _ = self.en_pin.set_low();
        Ok(buf)
     
        
    }

}
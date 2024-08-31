//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]


mod senseair;
use core::cell::RefCell;
use core::time::Duration;
use cortex_m::interrupt::{self, Mutex};
use cortex_m::delay::Delay;

use rp_pico::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use embedded_hal::blocking::i2c::{Read, Write};

use rp_pico::hal::fugit::RateExtU32;
use bme680::*;

use rp_pico::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    I2C,
    pac::{I2C1,I2C0}, 
    sio::Sio,
    watchdog::Watchdog,
    gpio::{FunctionI2C,FunctionI2c,Pin,OutputOverride,PinState,PullUp,PullDown,bank0::{Gpio16,Gpio17,Gpio18,Gpio19,Gpio20,Gpio21},FunctionSio,SioOutput,SioInput},
};
use senseair::Sunrise;

const ADDRESS: u8 = 0x68;
//static mut GLOBAL_DELAY:Option<cortex_m::delay::Delay> = None;
static mut GLOBAL_DELAY: Option<Delay> = None;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    //let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    unsafe{
        GLOBAL_DELAY = Some(Delay::new(core.SYST, clocks.system_clock.freq().to_Hz()));
    }
    

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

        // Configure two pins as being I²C, not GPIO
    let sda: Pin<_, FunctionI2C, _> = pins.gpio18.reconfigure();
    let scl: Pin<_, FunctionI2C, _> = pins.gpio19.reconfigure();

    let sda0: Pin<_, FunctionI2C, _> = pins.gpio16.reconfigure();
    let scl0: Pin<_, FunctionI2C, _> = pins.gpio17.reconfigure();
   

    let i2c =  rp_pico::hal::I2C::i2c1(
        pac.I2C1,
        sda,
        scl, // Try `not_an_scl_pin` here
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let i2c0 =  rp_pico::hal::I2C::i2c0(
        pac.I2C0,
        sda0,
        scl0, // Try `not_an_scl_pin` here
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
   
    
    let en_pin = pins.gpio20.into_push_pull_output_in_state(PinState::Low);
    let nrdy_pin =pins.gpio21.into_pull_up_input();

    //let mut  test:Option<Sunrise<rp_pico::hal::I2C<pac::I2C1, (Pin<rp_pico::hal::gpio::bank0::Gpio18, rp_pico::hal::gpio::FunctionI2c, rp_pico::hal::gpio::PullUp>, Pin<rp_pico::hal::gpio::bank0::Gpio19, rp_pico::hal::gpio::FunctionI2c, rp_pico::hal::gpio::PullUp>)>, cortex_m::delay::Delay, Pin<rp_pico::hal::gpio::bank0::Gpio20, rp_pico::hal::gpio::FunctionSio<rp_pico::hal::gpio::SioOutput>, rp_pico::hal::gpio::PullDown>, rp_pico::hal::gpio::AsInputPin<rp_pico::hal::gpio::bank0::Gpio21, rp_pico::hal::gpio::FunctionNull, rp_pico::hal::gpio::PullDown>> > = None;
    let mut  test:Option<Sunrise<I2C<I2C1, (Pin<Gpio18, FunctionI2c, PullUp>, Pin<Gpio19, FunctionI2c, PullUp>)>, Delay, Pin<Gpio20, FunctionSio<SioOutput>, PullDown>, Pin<Gpio21, FunctionSio<SioInput>, PullUp>>> = None;
    let mut test1:Option<Bme680<I2C<I2C0, (Pin<Gpio16, FunctionI2c, PullUp>, Pin<Gpio17, FunctionI2c, PullUp>)>, Delay>> = None;
    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            test = Some(Sunrise::new(i2c, delay, en_pin, nrdy_pin));
        }
    }
    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            test1 = Some( Bme680::init(i2c0, delay, I2CAddress::Secondary).unwrap());
        }
    }
    let mut sensor_CO2 = test.take().unwrap();
    let mut bme680 = test1.take().unwrap();
    let settings = SettingsBuilder::new()
    .with_humidity_oversampling(OversamplingSetting::OS2x)
    .with_pressure_oversampling(OversamplingSetting::OS4x)
    .with_temperature_oversampling(OversamplingSetting::OS8x)
    .with_temperature_filter(IIRFilterSize::Size3)
    .with_gas_measurement(Duration::from_millis(1500), 320, 25)
    .with_temperature_offset(-2.2)
    .with_run_gas(true)
    .build();
    let profile_dur = bme680.get_profile_dur(&settings.0).unwrap();
    info!("Profile duration {:?}", profile_dur);
    info!("Setting sensor settings");

    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            bme680.set_sensor_settings(delay, settings).unwrap();
        }
    }
    info!("Setting forced power modes");
    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            bme680.set_sensor_mode(delay, PowerMode::ForcedMode).unwrap();
        }
    }
    let sensor_settings = bme680.get_sensor_settings(settings.1);
  
    sensor_CO2.init(false,true,false).unwrap();

    
    
    loop {

    let data = sensor_CO2.single_measurement_get().unwrap();
    
    let sensor_data = ((data[6] as u16) << 8) | (data[7] as u16);
    info!("Read CO2 data is {}",sensor_data);
    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            bme680.set_sensor_mode(delay, PowerMode::ForcedMode).unwrap();
        }
    }
    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            let (data, _state) = bme680.get_sensor_data(delay).unwrap();
            info!("Temperature {}°C", data.temperature_celsius());
            info!("Pressure {}hPa", data.pressure_hpa());
            info!("Humidity {}%", data.humidity_percent());
            info!("Gas Resistence {}Ω", data.gas_resistance_ohm());
        }
    }
    
    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            delay.delay_ms(15000);
        }
    }
    }
}



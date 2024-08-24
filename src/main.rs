//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]


mod senseair;
use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};


use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use embedded_hal::blocking::i2c::{Read, Write};

use rp_pico::hal::gpio::PinState;
use rp_pico::pac::adc::result;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
use bsp::hal::fugit::RateExtU32;

use lps22hb::*;
use lps22hb::interface::{I2cInterface,i2c::I2cAddress};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    gpio::{FunctionI2C, Pin,OutputOverride},
};
use senseair::Sunrise;

const ADDRESS: u8 = 0x68;
//static mut GLOBAL_DELAY:Option<cortex_m::delay::Delay> = None;
static mut GLOBAL_DELAY: Option<cortex_m::delay::Delay> = None;

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

    //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    unsafe{
        GLOBAL_DELAY = Some(cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz()));
    }
    

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

        // Configure two pins as being I²C, not GPIO
    let sda: Pin<_, FunctionI2C, _> = pins.gpio18.reconfigure();
    let scl: Pin<_, FunctionI2C, _> = pins.gpio19.reconfigure();
   

    let mut i2c =  bsp::hal::I2C::i2c1(
        pac.I2C1,
        sda,
        scl, // Try `not_an_scl_pin` here
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

   
    
    let en_pin = pins.gpio20.into_push_pull_output_in_state(PinState::Low);
    let nrdy_pin =pins.gpio21.as_input();

    let mut  test:Option<Sunrise<rp_pico::hal::I2C<pac::I2C1, (Pin<rp_pico::hal::gpio::bank0::Gpio18, rp_pico::hal::gpio::FunctionI2c, rp_pico::hal::gpio::PullUp>, Pin<rp_pico::hal::gpio::bank0::Gpio19, rp_pico::hal::gpio::FunctionI2c, rp_pico::hal::gpio::PullUp>)>, cortex_m::delay::Delay, Pin<rp_pico::hal::gpio::bank0::Gpio20, rp_pico::hal::gpio::FunctionSio<rp_pico::hal::gpio::SioOutput>, rp_pico::hal::gpio::PullDown>, rp_pico::hal::gpio::AsInputPin<rp_pico::hal::gpio::bank0::Gpio21, rp_pico::hal::gpio::FunctionNull, rp_pico::hal::gpio::PullDown>> > = None;

    unsafe {
        if let Some(delay) = GLOBAL_DELAY.as_mut() {
            test = Some(Sunrise::new(i2c, delay, en_pin, nrdy_pin));
        }
    }

    // let mut sensor_CO2 = Sunrise::new(i2c, & mut delay, en_pin, nrdy_pin);

    // sensor_CO2.init(false,true,false).unwrap();
    if let Some(value) = test{
        
    }
    
    
    loop {

 //   sensor_CO2.single_measurement_get().unwrap();

    }
}



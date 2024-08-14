//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]


mod senseair;
use core::borrow::BorrowMut;

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
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    gpio::{FunctionI2C, Pin,OutputOverride},
};
use senseair::Sunrise;

const ADDRESS: u8 = 0x68;

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

        // Configure two pins as being I²C, not GPIO
    let sda: Pin<_, FunctionI2C, _> = pins.gpio18.reconfigure();
    let scl: Pin<_, FunctionI2C, _> = pins.gpio19.reconfigure();
   

    let mut i2c = bsp::hal::I2C::i2c1(
        pac.I2C1,
        sda,
        scl, // Try `not_an_scl_pin` here
        10.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    
    
    
  
        // let not_an_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio20.rec
    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.

    // let mut led_pin = pins.led.into_push_pull_output();
    let mut en_pin = pins.gpio20.into_push_pull_output_in_state(PinState::Low);
    let mut nrdy_pin =pins.gpio21.as_input();

 
    let sensor_CO2 = Sunrise::new(i2c, delay, en_pin, nrdy_pin);
        
    loop {


    }
}



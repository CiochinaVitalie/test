//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::cell::RefCell;
use core::convert::Infallible;
use cortex_m::interrupt::Mutex;
use cortex_m::singleton;
use core::fmt::Write;
use embedded_alloc::LlffHeap as Heap;

use core::time::Duration;
use cortex_m::delay::Delay;

use rp_pico::entry;
use defmt::{unimplemented,info};
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use panic_probe as _;
use embedded_dma;
// use embedded_hal::blocking::i2c::{Read, Write};
use rp_pico::hal::fugit::{RateExtU32,Hertz};

use renderer::Rgb565Pixel;
use slint::platform::{software_renderer as renderer, PointerEventButton, WindowEvent};
// use cortex_m::prelude::*;
use display_interface_spi::SPIInterface;
use mipidsi;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};


use rp_pico::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    gpio::*,
    spi::{Spi,Enabled},
    timer::{Alarm,Alarm0},
    dma::{DMAExt, SingleChannel, WriteTarget,single_buffer},
    Timer
};

//static mut GLOBAL_DELAY:Option<cortex_m::delay::Delay> = None;
static mut GLOBAL_DELAY: Option<Delay> = None;
const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
const DISPLAY_SIZE: slint::PhysicalSize = slint::PhysicalSize::new(320, 240);
const SPI_ILI9341_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(42_000_000);
static ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));

pub type TargetPixel = Rgb565Pixel;

type SpiPins = (
    Pin<bank0::Gpio11,FunctionSpi,PullDown>,
    Pin<bank0::Gpio12,FunctionSpi,PullDown>,
    Pin<bank0::Gpio10,FunctionSpi,PullDown>,
);

type EnabledSpi = Spi<Enabled, pac::SPI1, SpiPins, 8>;
type SpiRefCell = RefCell<(EnabledSpi, Hertz<u32>)>;
type Display<DI, RST> = mipidsi::Display<DI, mipidsi::models::ILI9341Rgb565, RST>;

#[derive(Clone)]
struct SharedSpiWithFreq<CS> {
    refcell: &'static SpiRefCell,
    cs: CS,
    freq: Hertz<u32>,
}
impl<CS> ErrorType for SharedSpiWithFreq<CS> {
    type Error = <EnabledSpi as ErrorType>::Error;
}


impl<CS: OutputPin<Error = Infallible>> SpiDevice for SharedSpiWithFreq<CS> {
    #[inline]
    fn transaction(&mut self, operations: &mut [Operation<u8>]) -> Result<(), Self::Error> {
        let mut borrowed = self.refcell.borrow_mut();
        if borrowed.1 != self.freq {
            borrowed.0.flush()?;
            // the touchscreen and the LCD have different frequencies
            borrowed.0.set_baudrate(125_000_000u32.Hz(), self.freq);
            borrowed.1 = self.freq;
        }
        self.cs.set_low()?;
        for op in operations {
            match op {
                Operation::Read(words) => borrowed.0.read(words),
                Operation::Write(words) => borrowed.0.write(words),
                Operation::Transfer(read, write) => borrowed.0.transfer(read, write),
                Operation::TransferInPlace(words) => borrowed.0.transfer_in_place(words),
                Operation::DelayNs(_) => unimplemented!(),
            }?;
        }
        borrowed.0.flush()?;
        drop(borrowed);
        self.cs.set_high()?;
        Ok(())
    }
}

struct PartialReadBuffer(&'static mut [Rgb565Pixel], core::ops::Range<usize>);
unsafe impl embedded_dma::ReadBuffer for PartialReadBuffer {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const <Self as embedded_dma::ReadBuffer>::Word, usize) {
        let act_slice = &self.0[self.1.clone()];
        (act_slice.as_ptr() as *const u8, act_slice.len() * core::mem::size_of::<Rgb565Pixel>())
    }
}
enum PioTransfer<TO: WriteTarget, CH: SingleChannel> {
    Idle(CH, &'static mut [TargetPixel], TO),
    Running(single_buffer::Transfer<CH, PartialReadBuffer, TO>),
}

impl<TO: WriteTarget<TransmittedWord = u8>, CH: SingleChannel> PioTransfer<TO, CH> {
    fn wait(self) -> (CH, &'static mut [TargetPixel], TO) {
        match self {
            PioTransfer::Idle(a, b, c) => (a, b, c),
            PioTransfer::Running(dma) => {
                let (a, b, to) = dma.wait();
                (a, b.0, to)
            }
        }
    }
}

struct DrawBuffer<Display, PioTransfer, Stolen> {
    display: Display,
    buffer: &'static mut [TargetPixel],
    pio: Option<PioTransfer>,
    stolen_pin: Stolen,
}

impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = Infallible>,
        TO: WriteTarget<TransmittedWord = u8>,
        CH: SingleChannel,
        DC_: OutputPin<Error = Infallible>,
        CS_: OutputPin<Error = Infallible>,
    > renderer::LineBufferProvider
    for &mut DrawBuffer<Display<DI, RST>, PioTransfer<TO, CH>, (DC_, CS_)>
{
    type TargetPixel = TargetPixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [TargetPixel]),
    ) {
        render_fn(&mut self.buffer[range.clone()]);

        /* -- Send the pixel without DMA
        self.display.set_pixels(
            range.start as _,
            line as _,
            range.end as _,
            line as _,
            self.buffer[range.clone()]
                .iter()
                .map(|x| embedded_graphics::pixelcolor::raw::RawU16::new(x.0).into()),
        );
        return;*/

        // convert from little to big endian before sending to the DMA channel
        for x in &mut self.buffer[range.clone()] {
            *x = Rgb565Pixel(x.0.to_be())
        }
        let (ch, mut b, spi) = self.pio.take().unwrap().wait();
        core::mem::swap(&mut self.buffer, &mut b);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                core::iter::empty(),
            )
            .unwrap();

        self.stolen_pin.1.set_low().unwrap();
        self.stolen_pin.0.set_high().unwrap();
        let mut dma = rp_pico::hal::dma::single_buffer::Config::new(ch, PartialReadBuffer(b, range), spi);
        dma.pace(rp_pico::hal::dma::Pace::PreferSink);
        self.pio = Some(PioTransfer::Running(dma.start()));
        /*let (a, b, c) = dma.start().wait();
        self.pio = Some(PioTransfer::Idle(a, b.0, c));*/
    }
}
impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = Infallible>,
        TO: WriteTarget<TransmittedWord = u8> + embedded_hal_nb::spi::FullDuplex,
        CH: SingleChannel,
        DC_: OutputPin<Error = Infallible>,
        CS_: OutputPin<Error = Infallible>,
    > DrawBuffer<Display<DI, RST>, PioTransfer<TO, CH>, (DC_, CS_)>
{
    fn flush_frame(&mut self) {
        let (ch, b, mut spi) = self.pio.take().unwrap().wait();
        self.stolen_pin.1.set_high().unwrap();

        // After the DMA operated, we need to empty the receive FIFO, otherwise the touch screen
        // driver will pick wrong values.
        // Continue to read as long as we don't get a Err(WouldBlock)
        while !spi.read().is_err() {}

        self.pio = Some(PioTransfer::Idle(ch, b, spi));
    }
}
struct WriteToScreen<'a, D> {
    x: i32,
    y: i32,
    width: i32,
    style: MonoTextStyle<'a, Rgb565>,
    display: &'a mut D,
}

impl<'a, D: DrawTarget<Color = Rgb565>> Write for WriteToScreen<'a, D> {
    fn write_str(&mut self, mut s: &str) -> Result<(), core::fmt::Error> {
        while !s.is_empty() {
            let (x, y) = (self.x, self.y);
            let end_of_line = s
                .find(|c| {
                    if c == '\n' || self.x > self.width {
                        self.x = 0;
                        self.y += 1;
                        true
                    } else {
                        self.x += 1;
                        false
                    }
                })
                .unwrap_or(s.len());
            let (line, rest) = s.split_at(end_of_line);
            let sz = self.style.font.character_size;
            Text::new(line, Point::new(x * sz.width as i32, y * sz.height as i32), self.style)
                .draw(self.display)
                .map_err(|_| core::fmt::Error)?;
            s = rest.strip_prefix('\n').unwrap_or(rest);
        }
        Ok(())
    }
}

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

    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);    

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
    let rst = pins.gpio15.into_push_pull_output();
    let mut backlight = pins.gpio13.into_push_pull_output();

    let dc = pins.gpio8.into_push_pull_output();
    let cs = pins.gpio9.into_push_pull_output();

    let spi_sclk = pins.gpio10.into_function::<FunctionSpi>();
    let spi_mosi = pins.gpio11.into_function::<FunctionSpi>();
    let spi_miso = pins.gpio12.into_function::<FunctionSpi>();
    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        SPI_ILI9341_MAX_FREQ,
        &embedded_hal::spi::MODE_3,
    );

    let (dc_copy, cs_copy) =
        unsafe { (core::ptr::read(&dc as *const _), core::ptr::read(&cs as *const _)) };

    let stolen_spi = unsafe { core::ptr::read(&spi as *const _) };
    let spi = singleton!(:SpiRefCell = SpiRefCell::new((spi, 0.Hz()))).unwrap();
    let display_spi = SharedSpiWithFreq { refcell: spi, cs, freq: SPI_ILI9341_MAX_FREQ };
    let di = SPIInterface::new(display_spi, dc);

    // let di = SPIInterface::new(display_spi, dc);

    let mut display = mipidsi::Builder::new(mipidsi::models::ILI9341Rgb565, di)
        .reset_pin(rst)
        .display_size(DISPLAY_SIZE.height as _, DISPLAY_SIZE.width as _)
        .orientation(mipidsi::options::Orientation::new().rotate(mipidsi::options::Rotation::Deg90))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut timer)
        .unwrap();

    backlight.set_high().unwrap();

    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();


    cortex_m::interrupt::free(|cs| {
        ALARM0.borrow(cs).replace(Some(alarm0));
        TIMER.borrow(cs).replace(Some(timer));
    });

    unsafe {
        // pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let dma = pac.DMA.split(&mut pac.RESETS);

    let pio = PioTransfer::Idle(
        dma.ch0,
        vec![Rgb565Pixel::default(); DISPLAY_SIZE.width as _].leak(),
        stolen_spi,
    );

    let buffer_provider = DrawBuffer {
        display,
        buffer: vec![Rgb565Pixel::default(); DISPLAY_SIZE.width as _].leak(),
        pio: Some(pio),
        stolen_pin: (dc_copy, cs_copy),
    };

      
    loop 
    {}
}




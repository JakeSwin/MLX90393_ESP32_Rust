#![no_std]
#![no_main]

use esp_backtrace as _;
use critical_section::Mutex;
use esp_println::println;
use postcard::to_slice_cobs;
use serde::Serialize;
use core::{cell::RefCell, fmt::Write, borrow::BorrowMut};
use hal::{clock::ClockControl, gpio::{Event, Gpio18, Input, PullUp, IO}, interrupt, peripherals::{self, Peripherals}, prelude::*, macros::ram, xtensa_lx, Delay, i2c::I2C, Uart, Rtc, uart};
use itertools::Itertools;
use core::sync::atomic::{AtomicBool, Ordering};
use hal::gpio::{Floating, GpioPin, Output, PushPull};
use hal::uart::config::{Config, Parity, StopBits};
use hal::uart::config::DataBits::DataBits8;
use hal::uart::TxRxPins;

#[derive(Debug, Clone, Copy, Serialize)]
struct SensorSample {
    t: u16,
    x: u16,
    y: u16,
    z: u16,
    timestamp: u64,
}

impl SensorSample {
    const fn empty() -> SensorSample {
        SensorSample { t: 0, x: 0, y: 0, z: 0, timestamp: 0 }
    }

    fn new(t: u16, x: u16, y: u16, z: u16, timestamp: u64) -> SensorSample {
        SensorSample { t, x, y, z, timestamp }
    }
}

static INT: Mutex<RefCell<Option<Gpio18<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static READ: Mutex<RefCell<AtomicBool>> = Mutex::new(RefCell::new(AtomicBool::new(false)));
// static SAMPLES: Mutex<RefCell<[SensorSample; 50]>> = Mutex::new(RefCell::new([SensorSample::empty(); 50]));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let rtc = Rtc::new(peripherals.RTC_CNTL);
    let config = Config {
        baudrate: 115200,
        data_bits: DataBits8,
        parity: Parity::ParityEven,
        stop_bits: StopBits::STOP1
    };
    // let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);
    type Pins<'a> = TxRxPins<'a, GpioPin<Output<PushPull>, 2>, GpioPin<Input<Floating>, 0>>;
    let mut uart0 = Uart::new_with_config(peripherals.UART0, Some(config), None::<Pins<'_>>, &clocks, &mut system.peripheral_clock_control);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio5,
        io.pins.gpio4,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );
    let mut int = io.pins.gpio18.into_pull_up_input();
    int.listen(Event::RisingEdge);

    critical_section::with(|cs| INT.borrow_ref_mut(cs).replace(int));

    println!("Hello world!");
    // Store Responses
    let mut res_buff = [0u8; 1];
    // Send EX command
    i2c.write_read(0x18, &[0x80], &mut res_buff).ok();
    println!("Exit Response: {:02x?}", res_buff);
    delay.delay_ms(100u32);
    // Send RT command
    i2c.write_read(0x18, &[0xF0], &mut res_buff).ok();
    println!("Reset Response: {:02x?}", res_buff);
    delay.delay_ms(100u32);
    // Send Start Burst Mode command
    i2c.write_read(0x18, &[0x1F], &mut res_buff).ok();
    println!("Burst Response: {:02x?}", res_buff);
    delay.delay_ms(100u32);

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority2).unwrap();

    let mut offset_x_buff = [0u8; 9];
    let mut count = 0;
    let mut samples: [SensorSample; 30] = [SensorSample::empty(); 30];
    loop {
        let _ = critical_section::with(|cs| {
            let mut binding = READ.borrow_ref_mut(cs);
            let r = binding.borrow_mut();
            if !r.load(Ordering::SeqCst) {
                return false;
            }
            i2c.write_read(0x18, &[0x4F], &mut offset_x_buff).ok();

            let buff_16 = offset_x_buff.map(u16::from);
            let t = (buff_16[1] << 8) | buff_16[2];
            let x = (buff_16[3] << 8) | buff_16[4];
            let y = (buff_16[5] << 8) | buff_16[6];
            let z = (buff_16[7] << 8) | buff_16[8];
            let time = rtc.get_time_us();

            samples[count] = SensorSample::new(t, x, y, z, time);
            *r.get_mut() = false;

            if count < 29 {
                count += 1;
            } else {
                let mut buff = [0u8; 525];
                to_slice_cobs(&samples, &mut buff).ok();
                writeln!(uart0, "{:02x}", buff.iter().format("")).ok();
                count = 0;
            }
            return true;
        });
    }
}

#[ram]
#[interrupt]
unsafe fn GPIO() {
    critical_section::with(|cs| {
        *READ.borrow_ref_mut(cs).borrow_mut().get_mut() = true;
        INT
            .borrow_ref_mut(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
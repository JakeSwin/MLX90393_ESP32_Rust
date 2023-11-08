#![no_std]
#![no_main]

use esp_backtrace as _;
use critical_section::Mutex;
use esp_println::println;
use postcard::to_slice_cobs;
use serde::Serialize;
use core::{cell::RefCell, fmt::Write};
use hal::{
    clock::ClockControl,
    gpio::IO,
    interrupt,
    peripherals::{self, Peripherals, UART0},
    prelude::*,
    Delay,
    i2c::I2C,
    uart::config::AtCmdConfig,
    Uart,
    Rtc
};
use itertools::Itertools;
use heapless::String;
use core::str;

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

static SAMPLES: Mutex<RefCell<[SensorSample; 50]>> = Mutex::new(RefCell::new([SensorSample::empty(); 50]));
static SERIAL: Mutex<RefCell<Option<Uart<UART0>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let rtc = Rtc::new(peripherals.RTC_CNTL);


    let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, b'#', None));
    uart0.set_rx_fifo_full_threshold(30).unwrap();
    uart0.listen_at_cmd();
    uart0.listen_rx_fifo_full();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio5,
        io.pins.gpio4,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    println!("Hello world!");
    let mut res_buff = [0u8; 1];
    // Send EX command
    i2c.write_read(0x18, &[0x80], &mut res_buff).ok();
    println!("Exit Response: {:02x?}", res_buff);
    delay.delay_ms(100u32);
    // Send RT command
    i2c.write_read(0x18, &[0xF0], &mut res_buff).ok();
    println!("Reset Response: {:02x?}", res_buff);
    delay.delay_ms(100u32);
    // Send Start Single Measurement Mode command
    // i2c.write_read(0x18, &[0x3F], &mut res_buff).ok();
    // println!("Single Measurement Response: {:02x?}", res_buff);
    // Send Start Burst Mode command
    i2c.write_read(0x18, &[0x1F], &mut res_buff).ok();
    println!("Burst Response: {:02x?}", res_buff);
    delay.delay_ms(100u32);

    interrupt::enable(
        peripherals::Interrupt::UART0,
        interrupt::Priority::Priority2
    ).unwrap();

    critical_section::with(|cs| SERIAL.borrow_ref_mut(cs).replace(uart0));

    let mut offset_x_buff = [0u8; 9];
    let mut count = 0;
    loop {
        i2c.write_read(0x18, &[0x4F], &mut offset_x_buff).ok();
        let time = rtc.get_time_us();
        // i2c.write_read(0x18, &[0x04 << 2], &mut offset_x_buff).ok();
        let buff_16 = offset_x_buff.map(u16::from);
        let t = (buff_16[1] << 8) | buff_16[2];
        let x = (buff_16[3] << 8) | buff_16[4];
        let y = (buff_16[5] << 8) | buff_16[6];
        let z = (buff_16[7] << 8) | buff_16[8];
        // println!("{:02x?}", offset_x_buff);
        println!("t: {}, x: {}, y: {}, z: {}", t, x, y, z);

        critical_section::with(|cs| {
            let mut samples = SAMPLES.borrow_ref_mut(cs);
            let samples = samples.as_mut();

            samples[count] = SensorSample::new(t, x, y, z, time);
        });

        if count < 49 {
            count += 1;
        } else {
            count = 0;
        }

        delay.delay_ms(100u32);
    }
}

#[interrupt]
fn UART0() {
    critical_section::with(|cs| {
        let mut serial = SERIAL.borrow_ref_mut(cs);
        let serial = serial.as_mut().unwrap();

        let mut cnt = 0;
        while let nb::Result::Ok(_c) = serial.read() {
            cnt += 1;
        }
        writeln!(serial, "Read {} bytes", cnt).ok();

        // writeln!(
        //     serial,
        //     "Interrupt AT-CMD: {} RX-FIFO-FULL: {}",
        //     serial.at_cmd_interrupt_set(),
        //     serial.rx_fifo_full_interrupt_set()
        // ).ok();

        let binding = SAMPLES.borrow_ref(cs);
        let samples = binding.as_ref();
        let mut buff = [0u8; 500];
        // let mut buff: Vec<u8, 500> = Vec::new();
        to_slice_cobs(samples, &mut buff).ok();
        // str::from_utf8(&buff).unwrap();
        writeln!(serial, "{:02x}", buff.iter().format("")).ok();
        // serial.write_str(str::from_utf8(&buff).unwrap()).ok();
        // serial.write_bytes(str::from_utf8(&buff).unwrap()).ok();

        serial.reset_at_cmd_interrupt();
        serial.reset_rx_fifo_full_interrupt();
    });
}
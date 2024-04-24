#![no_std]
#![no_main]

use lunasoc_pac as pac;
use lunasoc_hal as hal;

use hal::hal::delay::DelayUs;
use hal::Serial;
use hal::Timer;

use log::{debug, info};

use panic_halt as _;
use riscv_rt::entry;

#[allow(unused_imports)]
use micromath::F32Ext;

#[riscv_rt::pre_init]
unsafe fn pre_main() {
    pac::cpu::vexriscv::flush_icache();
    pac::cpu::vexriscv::flush_dcache();
}

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let leds = &peripherals.LEDS;

    // initialize logging
    let serial = Serial::new(peripherals.UART);
    hello_rust::log::init(serial);

    let mut timer = Timer::new(peripherals.TIMER, pac::clock::sysclk());
    let mut counter = 0;
    let mut direction = true;
    let mut led_state = 0b110000;

    info!("Peripherals initialized, entering main loop.");

    const HRAM_BASE: usize = 0x20000000;

    unsafe {
        let hram_ptr = HRAM_BASE as *mut u32;

        timer.enable();
        timer.set_timeout_ticks(0xFFFFFFFF);

        let start = timer.counter();

        //const n: isize = 1024*1024*1;
        //const n: isize = 64;
        const n: isize = 720*720/4;

        let mut cnt_x: isize = 0;

        for i in 0..n {
            hram_ptr.offset(i).write_volatile(0);
        }

        loop {



            /*

            info!("up");

            for i in 0..n {
                hram_ptr.offset(i).write_volatile(0);
            }

            info!("dn");

            for i in 0..n {
                hram_ptr.offset(i).write_volatile(0xffffffffu32);
            }

            */

            //info!("wht");

            /*
            for i in 0..n {
                hram_ptr.offset(i).write_volatile(0);
            }
            */

            for x in 0..720 {
                let y = (f32::sin((x as f32)/50.0f32) * 100.0f32 + 360.0f32) as isize;
                hram_ptr.offset(y*(720/4) + (x + 720/4)/4).write_volatile(0xffffffffu32);
                timer.delay_ms(1).unwrap();
            }


            for x in 0..720 {
                let y = (f32::sin((x as f32)/50.0f32) * 100.0f32 + 360.0f32) as isize;
                hram_ptr.offset(y*(720/4) + (x + 720/4)/4).write_volatile(0);
                timer.delay_ms(1).unwrap();
            }

            /*
            cnt_x += 1;
            if (cnt_x > 720-1) {
                cnt_x = 0;
            }
            */

            /*
            timer.delay_ms(500).unwrap();
            */

        }

        let endwrite = timer.counter();

        for i in 0..n {
            let got = hram_ptr.offset(i).read_volatile();
            if got != (0xDEAD0000u32 | i as u32) {
                info!("hyperram FL @ {:#x}, got {:#x}", i, got);
            }
        }

        let endread = timer.counter();

        let write_ticks = start-endwrite;
        let read_ticks = endwrite-endread;

        let sysclk = pac::clock::sysclk();

        info!("write speed {} KByte/sec", ((sysclk as u64) * (16*1024) as u64) / write_ticks as u64);

        info!("read speed {} KByte/sec", ((sysclk as u64) * (16*1024 as u64)) / (read_ticks as u64));

    }

    loop {
        timer.delay_ms(1000).unwrap();

        if direction {
            led_state >>= 1;
            if led_state == 0b000011 {
                direction = false;
                info!("left: {}", counter);
            }
        } else {
            led_state <<= 1;
            if led_state == 0b110000 {
                direction = true;
                debug!("right: {}", counter);
            }
        }

        leds.output().write(|w| unsafe { w.output().bits(led_state) });
        counter += 1;
    }
}

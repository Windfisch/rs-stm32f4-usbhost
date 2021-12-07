#![no_main]
#![no_std]

#![feature(type_alias_impl_trait)]
#![feature(pin_static_ref)]

mod null_waker;
mod usb_host;
mod driver;

use core::future::Future;
pub(crate) use core::pin::Pin;
use core::cell::RefCell;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::serial;
use core::fmt::Write;

use crate::hal::{prelude::*, stm32};

use core::task::{Poll, Context};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
	use core::mem::MaybeUninit;
	cortex_m::interrupt::disable();

	let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
	writeln!(tx, "Panic!").ok();
	writeln!(tx, "{}", _info).ok();

	loop {}
}



#[entry]
fn main() -> ! {
	if let (Some(dp), Some(cp)) = (
		stm32::Peripherals::take(),
		cortex_m::peripheral::Peripherals::take(),
	) {
		let rcc = dp.RCC.constrain();
		let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(84.mhz()).freeze();

		//let gpioc = dp.GPIOC.split();
		//let mut led = gpioc.pc13.into_push_pull_output();

		let gpioa = dp.GPIOA.split();
		let dm_pin = gpioa.pa11.into_alternate::<10>();
		let dp_pin = gpioa.pa12.into_alternate::<10>();

		// Create a delay abstraction based on SysTick
		let mut delay = hal::delay::Delay::new(cp.SYST, &clocks);


		let mut trigger_pin = gpioa.pa0.into_push_pull_output();
		trigger_pin.set_low();

		// Configure the USART
		let gpio_tx = gpioa.pa9.into_alternate();
		let gpio_rx = gpioa.pa10.into_alternate();
		let serial = serial::Serial::new(
			dp.USART1,
			(gpio_tx, gpio_rx),
			serial::config::Config::default().baudrate(115200.bps()),
			clocks
		).unwrap();
		let (mut tx, _rx) = serial.split();
		writeln!(tx, "hello world!").ok();


		let otg_fs_global = dp.OTG_FS_GLOBAL;
		let otg_fs_host = dp.OTG_FS_HOST;

		usb_host::usb_mainloop(otg_fs_global, otg_fs_host, tx, delay, trigger_pin);
	}

	loop {}
}


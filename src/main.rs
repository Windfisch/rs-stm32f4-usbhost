#![no_main]
#![no_std]

#![feature(type_alias_impl_trait)]
#![feature(pin_static_ref)]
#![feature(generators, generator_trait)]

mod coroutine;
mod usb_host;
mod driver;
mod transaction;
mod host_ext;

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
fn panic(info: &core::panic::PanicInfo) -> ! {
	use core::mem::MaybeUninit;
	cortex_m::interrupt::disable();

	let mut tx: serial::Tx<stm32::USART1> = unsafe { MaybeUninit::uninit().assume_init() };
	writeln!(tx, "Panic!").ok();
	writeln!(tx, "{}", info).ok();

	loop {}
}


use usb_host::{UsbHost, UsbHostCoroutine};

static mut USB_HOST: Option<UsbHost> = None;
static mut USB_HOST_COROUTINE: Option<UsbHostCoroutine> = None;


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
		let delay = hal::delay::Delay::new(cp.SYST, &clocks);


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

		use driver::DriverDescriptor;
		let host = usb_host::UsbHost::new(
			otg_fs_global,
			otg_fs_host,
			dp_pin,
			dm_pin,
			delay,
			trigger_pin
		);
		let mut usb_host_coroutine = host.make_coroutine();
		let mut usb_host_coroutine_pinned = unsafe { Pin::new_unchecked(&mut usb_host_coroutine) };

		let midi_driver_descriptor = driver::MidiDriverDescriptor{};
		let instance = midi_driver_descriptor.create_instance();
		let pinned_driver_instance = unsafe { Pin::new_unchecked(&instance) };

		loop {
			host.poll(usb_host_coroutine_pinned.as_mut(), &[]);
			//host.poll(usb_host_coroutine_pinned.as_mut(), &[pinned_driver_instance]);
		}

		//pinned_driver_instance.data().send([1,2,3,4]).unwrap();
	}

	loop {}
}


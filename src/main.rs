#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::serial;
use core::fmt::Write;

use crate::hal::{prelude::*, stm32};

struct fnord { foo: u32 }

#[entry]
fn main() -> ! {
	if let (Some(dp), Some(cp)) = (
		stm32::Peripherals::take(),
		cortex_m::peripheral::Peripherals::take(),
	) {
		// Set up the system clock. We want to run at 48MHz for this one.
		let rcc = dp.RCC.constrain();
		let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(84.mhz()).freeze();

		// Set up the LED. On the Nucleo-446RE it's connected to pin PA5.
		let gpioc = dp.GPIOC.split();
		let mut led = gpioc.pc13.into_push_pull_output();

		let gpioa = dp.GPIOA.split();
		let dm_pin = gpioa.pa11.into_alternate_af10();
		let dp_pin = gpioa.pa12.into_alternate_af10();

		// Create a delay abstraction based on SysTick
		let mut delay = hal::delay::Delay::new(cp.SYST, clocks);


		// Configure the USART
		let gpio_tx = gpioa.pa9.into_alternate_af7();
		let gpio_rx = gpioa.pa10.into_alternate_af7();

		let serial = serial::Serial::usart1(
			dp.USART1,
			(gpio_tx, gpio_rx),
			serial::config::Config::default().baudrate(115200.bps()),
			clocks
		).unwrap();
		let (mut tx, _rx) = serial.split();
		writeln!(tx, "hello world!").ok();


		unsafe {
			// enable OTG clock
			let rcc = &(*crate::stm32::RCC::ptr());
			rcc.ahb2enr.modify(|_,w| { w.otgfsen().set_bit() } );

			/*
			// soft reset
			writeln!(tx, "resetting").ok();
			dp.OTG_FS_GLOBAL.grstctl.write(|w| w.csrst().set_bit());
			while dp.OTG_FS_GLOBAL.grstctl.read().csrst().bit() {}
			writeln!(tx, "reset done").ok();
			delay.delay_ms(1_u32); // wait at least 3 PHY clocks

			// wait for AHB idle (docs for CSRST require this)
			writeln!(tx, "waiting for AHBIDL").ok();
			while !dp.OTG_FS_GLOBAL.grstctl.read().ahbidl().bit() {}
			*/


			// "Core initialization" step 1
			dp.OTG_FS_GLOBAL.gahbcfg.write(|w| w.gint().set_bit());
			// FIXME rxflvl, txfifo empty level
			//dp.OTG_FS_GLOBAL.gusbcfg.write(|w| w.physel().set_bit()); // this bit is always 1

			// "Core initialization" step 2
			dp.OTG_FS_GLOBAL.gusbcfg.write(|w| w
				.hnpcap().clear_bit()
				.srpcap().clear_bit()
				//.trdt().bits(0xF) // FIXME can/should be smaller
				//.tocal().bits(8) // FIXME same
				.fdmod().clear_bit() // must wait for 25ms before the change takes effect
				.fhmod().set_bit() // must wait for 25ms before the change takes effect
			);
		
			/*
			// configure vbus sensing
			dp.OTG_FS_GLOBAL.gccfg.modify(|_, w| w
				.vbusasen().set_bit()
				//.vbusbsen().set_bit()
				.pwrdwn().set_bit()
			);
			dp.OTG_FS_GLOBAL.gccfg.modify(|r, w| w.bits( r.bits() | (1<<21) )); // NOVBUSSENS
			*/

			// "Core initialization" step 3
			dp.OTG_FS_GLOBAL.gintmsk.write(|w| w // FIXME FIXME FIXME the C code has them all zero
				.otgint().set_bit()
				.mmism().set_bit()
				.sofm().set_bit()
//				.prtim().set_bit() // FIXME: "only accessible in host mode", why is this inaccessible then?!
			);
			// "Host initialization" step 1
			dp.OTG_FS_GLOBAL.gintmsk.modify(|r, w| w.bits( r.bits() | (1<<24) )); // PRTIM


			// "Host initialization" step 2
			dp.OTG_FS_HOST.hcfg.write(|w| w.fslspcs().bits(1)); // 48MHz
			dp.OTG_FS_HOST.hcfg.modify(|r, w| w.bits( r.bits() | (1<<2) )); // FSLSS
			
			
			// "Host initialization" steps 3-4: enable PPWR and wait for PCDET
			writeln!(tx, "setting PPWR and waiting for PCDET");
			dp.OTG_FS_HOST.hprt.write(|w| w.ppwr().set_bit()); // FIXME unsure if that's needed
			
			writeln!(tx, "> {}, {:08x}, {:08x}", dp.OTG_FS_HOST.hprt.read().ppwr().bit(), dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
			writeln!(tx, "{}", dp.OTG_FS_HOST.hprt.read().bits());
			while !dp.OTG_FS_HOST.hprt.read().pcdet().bit() {}
			writeln!(tx, "got PCDET!");
			writeln!(tx, "> {}, {:08x}, {:08x}", dp.OTG_FS_HOST.hprt.read().ppwr().bit(), dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
			
			// "Host initialization" steps 5-9
			writeln!(tx, "resetting the port and waiting for PENCHNG");
			dp.OTG_FS_HOST.hprt.modify(|_,w| w.prst().set_bit());
			delay.delay_ms(15_u32);
			dp.OTG_FS_HOST.hprt.modify(|_,w| w.prst().clear_bit());
			delay.delay_ms(15_u32);

			while !dp.OTG_FS_HOST.hprt.read().penchng().bit() {
				writeln!(tx, "> {}, {:08x}, {:08x}", dp.OTG_FS_HOST.hprt.read().ppwr().bit(), dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
				delay.delay_ms(100_u32);
				//writeln!(tx, "{}, {}", dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
				//dp.OTG_FS_GLOBAL.gintsts.modify(|r,w| w.bits(r.bits()));
			}
			writeln!(tx, "enumerated speed is {}", dp.OTG_FS_HOST.hprt.read().pspd().bits());
			
			// "Host initialization" step 10: program hfir
			// TODO: is that needed? seems to autoselect a good value

			// "Host initialization" step 11: set PHY clock
			// TODO: is that needed? we already set 1 = 48MHz here. reset port if changed.
			//dp.OTG_FS_HOST.hcfg.write(|w| w.fslspcs().bits(1)); // 48MHz

			// "Host initialization" steps 12-14
			dp.OTG_FS_GLOBAL.grxfsiz.write(|w| w
				.rxfd().bits(64) // 64 32bit words RX fifo size
			);
			dp.OTG_FS_GLOBAL.hnptxfsiz_mut().write(|w| w
				.nptxfd().bits(64)
				.nptxfsa().bits(64)
			);
			dp.OTG_FS_GLOBAL.hptxfsiz.write(|w| w
				.ptxfsiz().bits(64)
				.ptxsa().bits(64+64)
			);

			// ---- done ----



			delay.delay_ms(250_u32);

			dp.OTG_FS_GLOBAL.gintmsk.write(|w| w.bits(0));
			dp.OTG_FS_GLOBAL.gintsts.write(|w| w.bits(!0));


			writeln!(tx, "gintsts = {:08x}", dp.OTG_FS_GLOBAL.gintsts.read().bits());
			writeln!(tx, "hprt = {:08x}", dp.OTG_FS_HOST.hprt.read().bits());
			delay.delay_ms(300_u32);
			writeln!(tx, "hprt = {:08x}", dp.OTG_FS_HOST.hprt.read().bits());
			loop {
				if dp.OTG_FS_GLOBAL.gintsts.read().sof().bit() {
					dp.OTG_FS_GLOBAL.gintsts.write(|w| w.sof().set_bit());
					writeln!(tx, "sof");
				}
				if dp.OTG_FS_GLOBAL.gintsts.read().otgint().bit() {
					let val = dp.OTG_FS_GLOBAL.gotgint.read().bits();
					writeln!(tx, "otg {:08x}", val);
					dp.OTG_FS_GLOBAL.gotgint.write(|w| w.bits(val));
				}
				if dp.OTG_FS_GLOBAL.gintsts.read().hprtint().bit() {
					let val = dp.OTG_FS_HOST.hprt.read().bits();
					writeln!(tx, "hprt {:08x}", val);
					dp.OTG_FS_HOST.hprt.modify(|r,w| w.bits( r.bits() | 4 ));
					//dp.OTG_FS_HOST.hprt.write(|w| w.bits((val & 0x22)|4));
				}
				if dp.OTG_FS_GLOBAL.gintsts.read().discint().bit() {
					dp.OTG_FS_GLOBAL.gintsts.write(|w| w.discint().set_bit());
					writeln!(tx, "discint");
					let val = dp.OTG_FS_HOST.hprt.read().bits();
					writeln!(tx, "hprt {:08x}", val);
				}
			}


			// --------
			if false {
			delay.delay_ms(25_u32);






			// TODO program FSLSS to 1 on devices that support it

			delay.delay_ms(100_u32);
			while !dp.OTG_FS_HOST.hprt.read().penchng().bit() {
				writeln!(tx, "{}, {}", dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
				dp.OTG_FS_GLOBAL.gintsts.modify(|r,w| w.bits(r.bits()));
			}
			dp.OTG_FS_GLOBAL.gahbcfg.write(|w| w // TODO: the C code does this at the end
				.gint().set_bit()
				.ptxfelvl().clear_bit()
			);
			//dp.OTG_FS_GLOBAL.gintsts.write(|w| w.rxflvl().set_bit()); FIXME p763, 1.
			writeln!(tx, "done");

			let speed = dp.OTG_FS_HOST.hprt.read().pspd().bits();
			writeln!(tx, "speed = {}", speed);

			// FIXME: 10. Program the HFIR register with a value corresponding to the selected PHY clock 1

			// TODO: do we need a port reset?
			}
		}
		writeln!(tx, "all done :)");

		loop {
			// On for 1s, off for 1s.
			led.set_high().unwrap();
			delay.delay_ms(1000_u32);
			led.set_low().unwrap();
			delay.delay_ms(1000_u32);
		}
	}

	loop {}
}

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
			let rcc = &(*crate::stm32::RCC::ptr());
			rcc.ahb2enr.modify(|_,w| { w.otgfsen().set_bit() } );

			dp.OTG_FS_GLOBAL.gahbcfg.write(|w| w.gint().clear_bit());
			dp.OTG_FS_GLOBAL.gusbcfg.write(|w| w.physel().set_bit());

		/*	
			writeln!(tx, "resetting").ok();
			dp.OTG_FS_GLOBAL.grstctl.write(|w| w.csrst().set_bit());
			while dp.OTG_FS_GLOBAL.grstctl.read().csrst().bit() {}
			writeln!(tx, "reset done").ok();
			delay.delay_ms(1_u32); // FIXME wait 3 PHY clocks
			*/
			

			writeln!(tx, "waiting for AHBIDL").ok();
			while !dp.OTG_FS_GLOBAL.grstctl.read().ahbidl().bit() {}
			
			dp.OTG_FS_GLOBAL.gccfg.write(|w| w
				.vbusasen().set_bit()
				//.vbusbsen().set_bit()
				.pwrdwn().set_bit()
			);
			dp.OTG_FS_GLOBAL.gccfg.modify(|r, w| w.bits( r.bits() | (1<<21) )); // NOVBUSSENS

			delay.delay_ms(250_u32);
			// FIXME rmeove these 2
			dp.OTG_FS_HOST.hcfg.write(|w| w.fslspcs().bits(1)); // 48MHz
			//dp.OTG_FS_HOST.hcfg.modify(|r, w| w.bits( r.bits() | (1<<2) )); // FSLSS




			dp.OTG_FS_GLOBAL.gahbcfg.write(|w| w // TODO: the C code doesn't do this here
				.gint().set_bit()
			);
			dp.OTG_FS_GLOBAL.gintmsk.write(|w| w // FIXME FIXME FIXME the C code has them all zero
				.otgint().set_bit()
				.mmism().set_bit()
				.sofm().set_bit()
//				.prtim().set_bit() // FIXME: "only accessible in host mode", why is this inaccessible then?!
			);
			dp.OTG_FS_GLOBAL.gintmsk.modify(|r, w| w.bits( r.bits() | (1<<24) )); // PRTIM


			delay.delay_ms(50_u32);

			dp.OTG_FS_GLOBAL.gusbcfg.write(|w| w
				.hnpcap().clear_bit()
				.srpcap().clear_bit()
				//.trdt().bits(0xF) // FIXME can/should be smaller
				//.tocal().bits(8) // FIXME same
				.fdmod().clear_bit() // must wait for 25ms before the change takes effect
				.fhmod().set_bit() // must wait for 25ms before the change takes effect
			);

			
			delay.delay_ms(250_u32);

			writeln!(tx, "PCGCCTL = {:08x}", dp.OTG_FS_PWRCLK.pcgcctl.read().bits());
			dp.OTG_FS_PWRCLK.pcgcctl.write(|w| w.bits(0));


			dp.OTG_FS_HOST.hcfg.write(|w| w.fslspcs().bits(1)); // 48MHz
			dp.OTG_FS_HOST.hcfg.modify(|r, w| w.bits( r.bits() | (1<<2) )); // FSLSS

			// TODO maybe remove that reset
			dp.OTG_FS_HOST.hprt.write(|w| w.prst().set_bit());
			delay.delay_ms(15_u32);
			dp.OTG_FS_HOST.hprt.write(|w| w.prst().clear_bit());
			delay.delay_ms(15_u32);



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



			dp.OTG_FS_GLOBAL.gintmsk.write(|w| w.bits(0));
			dp.OTG_FS_GLOBAL.gintsts.write(|w| w.bits(!0));

			writeln!(tx, "setting PPWR and waiting for PCDET");
			dp.OTG_FS_HOST.hprt.write(|w| w.ppwr().set_bit()); // FIXME unsure if that's needed
			
			delay.delay_ms(200_u32); // FIXME
			
			dp.OTG_FS_GLOBAL.gahbcfg.write(|w| w.gint().set_bit());


			writeln!(tx, "> {}, {:08x}, {:08x}", dp.OTG_FS_HOST.hprt.read().ppwr().bit(), dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
			writeln!(tx, "{}", dp.OTG_FS_HOST.hprt.read().bits());
			while !dp.OTG_FS_HOST.hprt.read().pcdet().bit() {}
			writeln!(tx, "got PCDET!");
			writeln!(tx, "> {}, {:08x}, {:08x}", dp.OTG_FS_HOST.hprt.read().ppwr().bit(), dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());

			delay.delay_ms(10_u32);

			dp.OTG_FS_HOST.hprt.write(|w| w.ppwr().set_bit()); // FIXME unsure if that's needed
			delay.delay_ms(200_u32);
			writeln!(tx, "resetting the port and waiting for PENCHNG");
			//dp.OTG_FS_HOST.hprt.modify(|r,w| w.bits( r.bits() | (1<<8)));
			dp.OTG_FS_HOST.hprt.modify(|_,w| w.prst().set_bit());
			delay.delay_ms(15_u32);
			//dp.OTG_FS_HOST.hprt.modify(|r,w| w.bits( r.bits() & !(1<<8)));
			dp.OTG_FS_HOST.hprt.modify(|_,w| w.prst().clear_bit());
			delay.delay_ms(15_u32);

			while !dp.OTG_FS_HOST.hprt.read().penchng().bit() {
				writeln!(tx, "> {}, {:08x}, {:08x}", dp.OTG_FS_HOST.hprt.read().ppwr().bit(), dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
				delay.delay_ms(100_u32);
				//writeln!(tx, "{}, {}", dp.OTG_FS_HOST.hprt.read().bits(), dp.OTG_FS_GLOBAL.gintsts.read().bits());
				//dp.OTG_FS_GLOBAL.gintsts.modify(|r,w| w.bits(r.bits()));
			}

			//delay.delay_ms(12_u32);

			writeln!(tx, "enumerated speed is {}", dp.OTG_FS_HOST.hprt.read().pspd().bits());

			// HFIR?
			// FSLSPCS and reset?


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

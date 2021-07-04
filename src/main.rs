#![feature(generators, generator_trait)]
#![no_main]
#![no_std]

use core::ops::Generator;
// Halt on panic
use panic_halt as _; // panic handler

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::serial;
use core::fmt::Write;

use crate::hal::{prelude::*, stm32};

#[derive(Debug)]
enum UsbLoop {
	Connected,
	Disconnected
}

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


		let otg_fs_global = dp.OTG_FS_GLOBAL;
		let otg_fs_host = dp.OTG_FS_HOST;

		let mut tx2 = unsafe { core::ptr::read(&tx) };

		let mut statemachine = || { unsafe {
			// enable OTG clock
			let rcc = &(*crate::stm32::RCC::ptr());
			rcc.ahb2enr.modify(|_,w| { w.otgfsen().set_bit() } );

			/*
			// soft reset
			writeln!(tx, "resetting").ok();
			otg_fs_global.grstctl.modify(|_,w| w.csrst().set_bit());
			while otg_fs_global.grstctl.read().csrst().bit() { yield None; }
			writeln!(tx, "reset done").ok();
			delay.delay_ms(1_u32); // wait at least 3 PHY clocks

			// wait for AHB idle (docs for CSRST require this)
			writeln!(tx, "waiting for AHBIDL").ok();
			while !otg_fs_global.grstctl.read().ahbidl().bit() { yield None; }
			*/

			// "Core initialization" step 1
			otg_fs_global.gahbcfg.modify(|_,w| w.gint().set_bit());
			// FIXME rxflvl, txfifo empty level
			//otg_fs_global.gusbcfg.modify(|_,w| w.physel().set_bit()); // this bit is always 1

			// "Core initialization" step 2
			otg_fs_global.gusbcfg.modify(|_,w| w
				.hnpcap().clear_bit()
				.srpcap().clear_bit()
				//.trdt().bits(0xF) // FIXME can/should be smaller
				//.tocal().bits(8) // FIXME same
				.fdmod().clear_bit() // must wait for 25ms before the change takes effect
				.fhmod().set_bit() // must wait for 25ms before the change takes effect
			);
		
			/*
			// configure vbus sensing
			otg_fs_global.gccfg.modify(|_, w| w
				.vbusasen().set_bit()
				//.vbusbsen().set_bit()
				.pwrdwn().set_bit()
			);
			otg_fs_global.gccfg.modify(|r, w| w.bits( r.bits() | (1<<21) )); // NOVBUSSENS
			*/

			// "Core initialization" step 3
			otg_fs_global.gintmsk.modify(|_,w| w // FIXME FIXME FIXME the C code has them all zero
				.otgint().set_bit()
				.mmism().set_bit()
				.sofm().set_bit()
//				.prtim().set_bit() // FIXME: "only accessible in host mode", why is this inaccessible then?!
			);
			// "Host initialization" step 1
			otg_fs_global.gintmsk.modify(|r, w| w.bits( r.bits() | (1<<24) )); // PRTIM


			// "Host initialization" step 2
			otg_fs_host.hcfg.modify(|_,w| w.fslspcs().bits(1)); // 48MHz
			otg_fs_host.hcfg.modify(|r, w| w.bits( r.bits() | (1<<2) )); // FSLSS
			
			
			loop {
				// "Host initialization" steps 3-4: enable PPWR and wait for PCDET
				writeln!(tx, "setting PPWR and waiting for PCDET").ok();
				otg_fs_host.hprt.modify(|_,w| w.ppwr().set_bit()); // FIXME unsure if that's needed
				while !otg_fs_host.hprt.read().pcdet().bit() { yield None; }
				writeln!(tx, "got PCDET!").ok();
				
				// "Host initialization" steps 5-9
				writeln!(tx, "resetting the port and waiting for PENCHNG").ok();
				otg_fs_host.hprt.modify(|_,w| w.prst().set_bit());
				delay.delay_ms(15_u32);
				otg_fs_host.hprt.modify(|_,w| w.prst().clear_bit());
				delay.delay_ms(15_u32);

				while !otg_fs_host.hprt.read().penchng().bit() { yield None; }
				writeln!(tx, "enumerated speed is {}", otg_fs_host.hprt.read().pspd().bits()).ok();
				
				// "Host initialization" step 10: program hfir
				// TODO: is that needed? seems to autoselect a good value

				// "Host initialization" step 11: set PHY clock
				// TODO: is that needed? we already set 1 = 48MHz here. reset port if changed.
				//otg_fs_host.hcfg.modify(|_,w| w.fslspcs().bits(1)); // 48MHz

				// "Host initialization" steps 12-14
				otg_fs_global.grxfsiz.modify(|_,w| w
					.rxfd().bits(64) // 64 32bit words RX fifo size
				);
				otg_fs_global.hnptxfsiz_mut().modify(|_,w| w
					.nptxfd().bits(64)
					.nptxfsa().bits(64)
				);
				otg_fs_global.hptxfsiz.modify(|_,w| w
					.ptxfsiz().bits(64)
					.ptxsa().bits(64+64)
				);

				// ---- done ----
				otg_fs_global.gintmsk.modify(|_,w| w.hcim().set_bit());
				// TODO maybe unmask NPTXFEM or so

				const N_CHANNELS: u32 = 8;
				otg_fs_host.haintmsk.write(|w| w.bits( (1<<N_CHANNELS)-1 ));

				macro_rules! hcint {
					($HCINT:ident, $HCINTMSK:ident) => {
							otg_fs_host.$HCINTMSK.write(|w| w
								.xfrcm().set_bit()
								.chhm().set_bit()
								.stallm().set_bit()
								.nakm().set_bit()
								.ackm().set_bit()
								.txerrm().set_bit()
								.bberrm().set_bit()
								.frmorm().set_bit()
								.dterrm().set_bit()
							);
							otg_fs_host.$HCINT.modify(|r,w| w.bits(r.bits()));
					}
				}
				hcint!(hcint0, hcintmsk0);
				hcint!(hcint1, hcintmsk1);
				hcint!(hcint2, hcintmsk2);
				hcint!(hcint3, hcintmsk3);
				hcint!(hcint4, hcintmsk4);
				hcint!(hcint5, hcintmsk5);
				hcint!(hcint6, hcintmsk6);
				hcint!(hcint7, hcintmsk7);




				// "host programming model"/"channel initialization"



				delay.delay_ms(250_u32);

				otg_fs_global.gintmsk.modify(|_,w| w.bits(0));
				otg_fs_global.gintsts.modify(|_,w| w.bits(!0));


				writeln!(tx, "gintsts = {:08x}", otg_fs_global.gintsts.read().bits()).ok();
				writeln!(tx, "hprt = {:08x}", otg_fs_host.hprt.read().bits()).ok();
				delay.delay_ms(300_u32);
				writeln!(tx, "hprt = {:08x}", otg_fs_host.hprt.read().bits()).ok();
				let mut sofcount: u32 = 0;

				yield Some(UsbLoop::Connected);

				loop {
					if otg_fs_global.gintsts.read().sof().bit() {
						otg_fs_global.gintsts.write(|w| w.sof().set_bit());
						write!(tx, "{:8}\x08\x08\x08\x08\x08\x08\x08\x08", sofcount);
						sofcount += 1;
					}

					while otg_fs_global.gintsts.read().rxflvl().bit() {
						writeln!(tx, "read").ok();
						// TODO do things
					}

					// OTGINT
					if otg_fs_global.gintsts.read().otgint().bit() {
						let val = otg_fs_global.gotgint.read().bits();
						writeln!(tx, "otg {:08x}", val).ok();
						otg_fs_global.gotgint.modify(|_,w| w.bits(val));
					}

					// HPRTINT
					if otg_fs_global.gintsts.read().hprtint().bit() {
						let val = otg_fs_host.hprt.read();
						writeln!(tx, "hprt {:08x}", val.bits()).ok();

						if val.penchng().bit() {
							writeln!(tx, "penchng, port enabled is {}", val.pena().bit()).ok();
						}

						if val.pocchng().bit() {
							writeln!(tx, "overcurrent").ok();
						}

						if val.pcdet().bit() {
							writeln!(tx, "port connect detected (pcdet)").ok();
						}
				
						writeln!(tx, "hprt before {:08x}", val.bits()).ok();
						otg_fs_host.hprt.modify(|r,w| w.bits(r.bits() & !4)); // do not set PENA???
						writeln!(tx, "hprt after {:08x}", val.bits()).ok();
					}

					// DISCINT
					if otg_fs_global.gintsts.read().discint().bit() {
						otg_fs_global.gintsts.write(|w| w.discint().set_bit());
						writeln!(tx, "disconnect (discint)").ok();
						yield Some(UsbLoop::Disconnected);
						break;
					}

					// MMIS
					if otg_fs_global.gintsts.read().mmis().bit() {
						otg_fs_global.gintsts.write(|w| w.mmis().set_bit());
						writeln!(tx, "mode mismatch (mmis)").ok();
					}

					// IPXFR
					if otg_fs_global.gintsts.read().ipxfr_incompisoout().bit(){
						otg_fs_global.gintsts.write(|w| w.ipxfr_incompisoout().set_bit());
						writeln!(tx, "ipxfr").ok();
					}

					// HCINT
					if otg_fs_global.gintsts.read().hcint().bit() {
						writeln!(tx, "hcint").ok();

						// TODO
					}

					delay.delay_ms(1_u32);
					yield None;
				}
			}
		}};

		loop {
			if let core::ops::GeneratorState::Yielded( Some(x) ) = core::pin::Pin::new(&mut statemachine).resume(())
			{
				writeln!(tx2, "<<<< {:?} >>>>", x);
			}
			 
		}

	}

	loop {}
}

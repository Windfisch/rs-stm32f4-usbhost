#![feature(generators, generator_trait)]
#![no_main]
#![no_std]

use core::ops::Generator;

use core::convert::TryInto;
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
	Disconnected,
	IgnoreThis
}

trait Fnord {
	fn hcintx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINT0;
	fn hccharx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0;
	fn hcintmskx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0;
	fn hctsizx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0;
}

impl Fnord for stm32f4xx_hal::pac::OTG_FS_HOST {
	fn hcintx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINT0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCINT0 = &self.hcint0;
		unsafe { &*(ptr.wrapping_add(i as usize * 0x20)) }
	}
	fn hcintmskx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0 = &self.hcintmsk0;
		unsafe { &*(ptr.wrapping_add(i as usize * 0x20)) }
	}
	fn hctsizx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0 = &self.hctsiz0;
		unsafe { &*(ptr.wrapping_add(i as usize * 0x20)) }
	}
	fn hccharx(&self, i: u32) -> &stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0 = &self.hcchar0;
		unsafe { &*(ptr.wrapping_add(i as usize * 0x20)) }
	}
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
		let dm_pin = gpioa.pa11.into_alternate_af10().internal_pull_down(true);
		let dp_pin = gpioa.pa12.into_alternate_af10().internal_pull_down(true);

		// Create a delay abstraction based on SysTick
		let mut delay = hal::delay::Delay::new(cp.SYST, &clocks);


		let mut trigger_pin = gpioa.pa0.into_push_pull_output();
		trigger_pin.set_low();

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
		
			
			// configure vbus sensing
			otg_fs_global.gccfg.modify(|_, w| w
			//	.vbusasen().set_bit()
			//	//.vbusbsen().set_bit()
				.pwrdwn().set_bit()
			);
			//otg_fs_global.gccfg.modify(|r, w| w.bits( r.bits() | (1<<21) )); // NOVBUSSENS
			

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

				otg_fs_global.gintmsk.modify(|_,w| w.nptxfem().set_bit());

//				const N_CHANNELS: u32 = 8;
//				otg_fs_host.haintmsk.write(|w| w.bits( (1<<N_CHANNELS)-1 ));

/*
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
*/
				// "host programming model"/"channel initialization"



				delay.delay_ms(250_u32);

				otg_fs_global.gintmsk.modify(|_,w| w.hcim().set_bit());
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
						//write!(tx, "{:8} {:08x}\r", sofcount, otg_fs_host.hcchar0.read().bits());
						write!(tx, "{:8} {:08x} {:08x}\r", sofcount, otg_fs_host.hfnum.read().bits(), otg_fs_global.gnptxsts.read().bits());
						//writeln!(tx, "hprt = {:08x}", otg_fs_host.hprt.read().bits()).ok();
						//write!(tx, "{:08x}\r", otg_fs_global.gintsts.read().bits());
						sofcount += 1;
					}

					let frame_number = otg_fs_host.hfnum.read().frnum().bits();

					while otg_fs_global.gintsts.read().srqint().bit() {
						writeln!(tx, "srqint").ok();
						otg_fs_global.gintsts.write(|w| w.srqint().set_bit());
						// TODO do things
					}

					while otg_fs_global.gintsts.read().rxflvl().bit() {
						let rxstsp = otg_fs_global.grxstsp_host().read();
						writeln!(tx, "#{}: read ch={} dpid={} bcnt={} pktsts={} {}", frame_number, rxstsp.chnum().bits(), rxstsp.dpid().bits(), rxstsp.bcnt().bits(), rxstsp.pktsts().bits(),
							match rxstsp.pktsts().bits() {
								2 => "IN data packet received",
								3 => "IN transfer completed",
								5 => "Data toggle error",
								7 => "Channel halted",
								_ => "(unknown)"
							}
						).ok();
						// TODO do things https://github.com/libusbhost/libusbhost/blob/master/src/usbh_lld_stm32f4.c#L322
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
							writeln!(tx, "#{}, penchng, port enabled is {}", frame_number, val.pena().bit()).ok();

							for i in 0..=1 {
								otg_fs_host.hcintx(i).write(|w| w.bits(!0));
								otg_fs_host.hcintmskx(i).write(|w| w
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
							}
							
							otg_fs_host.haintmsk.write(|w| w.bits(1<<0));
							
						

							let setup_packet = [
								0x00u8, // device, standard, host to device
								0x05, // set address
								0x01, 0x00, // address 1
								0x00, 0x00, // index
								0x00, 0x00, // length
							];

							otg_fs_host.hctsiz0.modify(|_, w| w
								.dpid().bits(3) // mdata. magically turns the OUT token into a SETUP token
								.pktcnt().bits(1)
								.xfrsiz().bits(8)
							);
							otg_fs_host.hcchar0.modify(|_, w| w
								.dad().bits(0)
								.mcnt().bits(1)
								.epdir().clear_bit()
								//.lsdev().set_bit() // TODO
								.epnum().bits(0) // 1 == in
								.eptyp().bits(0) // 0 == control
								.mpsiz().bits(64)
							);
							writeln!(tx, "gnptxsts = {:08x}, hptxfsiz = {:08x}", otg_fs_global.gnptxsts.read().bits(), otg_fs_global.hptxfsiz.read().bits());

							trigger_pin.set_high();
							unsafe {
								// NOTE: it does not matter where to write in the area 0x50001000 to 0x50001FFF. You can write your four-byte-chunks in ascending address order
								// (as memcpy would do), but you can also write them all to the same address (e.g. 0x50001000) or in descending addresses. I.e. reverse memcpy
								// would *not* cause the same result as memcpy
								//core::ptr::write_volatile((0x50001f00) as *mut u32, 0xdeadbeef);
								//writeln!(tx, "gnptxsts = {:08x}, hptxfsiz = {:08x}", otg_fs_global.gnptxsts.read().bits(), otg_fs_global.hptxfsiz.read().bits());

								// NOTE: setting pktcnt correctly is *not* needed to send a packet.
								// Instead, always ceil(xfrsiz / mpsiz) packets are sent; the first n-1 packets are mpsiz long, the last has the remaining length.
								// However, only after `pktcount` packets have been sent, a "transfer complete" interrupt is generated. So even if *sending* works
								// without setting pktcount, getting notified after the send completes would not work.

								// NOTE: the CHENA is pure user convenience. It does *not* control actual activation of the channel (which is triggered
								// by writing enough words to the FIFO). But the application will clear this bit after successfully transmitting `pktcnt` packets,
								// i.e. at the same time when the "transmission complete" interrupt is generated. If the user sets the bit, they can easily poll
								// which channels are free; however, the user *could* implement this bookkeeping using the transmission complete interrupt with their
								// own array, if they wanted to.
								otg_fs_host.hcchar0.modify(|_, w| w.chena().set_bit());
								core::ptr::write_volatile((0x50001000) as *mut [u8; 8], setup_packet);
							}
							while otg_fs_host.hcchar0.read().chena().bit() {
								//yield Some(UsbLoop::IgnoreThis);
							}
							trigger_pin.set_low();

							otg_fs_host.hctsiz0.modify(|_, w| w
								.dpid().bits(2)
								.pktcnt().bits(0)
								.xfrsiz().bits(0)
							);
							otg_fs_host.hcchar0.modify(|_, w| w
								.dad().bits(0)
								.mcnt().bits(1)
								.epdir().set_bit()
								//.lsdev().set_bit() // TODO
								.epnum().bits(0) // 1 == in
								.eptyp().bits(0) // 0 == control
								.mpsiz().bits(64)
							);
							otg_fs_host.hcchar0.modify(|_, w| w.chena().set_bit());
							//core::ptr::write_volatile((0x50001000) as *mut [u8; 8], setup_packet);
					
							while otg_fs_host.hcchar0.read().chena().bit() {
								writeln!(tx, "waiting for chena to become 0");
								while !otg_fs_global.gintsts.read().rxflvl().bit() {
									//writeln!(tx, "waiting for rxflvl");
								}
								writeln!(tx, "gotcha");
								while otg_fs_global.gintsts.read().rxflvl().bit() { // FIXME duplicated code
									let rxstsp = otg_fs_global.grxstsp_host().read();
									writeln!(tx, "#{}: read ch={} dpid={} bcnt={} pktsts={} {}", frame_number, rxstsp.chnum().bits(), rxstsp.dpid().bits(), rxstsp.bcnt().bits(), rxstsp.pktsts().bits(),
										match rxstsp.pktsts().bits() {
											2 => "IN data packet received",
											3 => "IN transfer completed",
											5 => "Data toggle error",
											7 => "Channel halted",
											_ => "(unknown)"
										}
									).ok();
									// TODO do things https://github.com/libusbhost/libusbhost/blob/master/src/usbh_lld_stm32f4.c#L322
								}
							}

							otg_fs_host.hcchar0.modify(|_, w| w.chdis().set_bit());

							trigger_pin.set_high();
							otg_fs_host.hctsiz1.modify(|_, w| w
								.dpid().bits(3)
								.pktcnt().bits(1)
								.xfrsiz().bits(8)
							);
							otg_fs_host.hcchar1.modify(|_, w| w
								.dad().bits(1)
								.mcnt().bits(1)
								.epdir().clear_bit()
								//.lsdev().set_bit() // TODO
								.epnum().bits(0) // 1 == in
								.eptyp().bits(0) // 0 == control
								.mpsiz().bits(64)
							);
							let get_descriptor_packet = [
								0x80u8, // device, standard, host to device
								0x06, // get descriptor
								0x00, 0x01, // descriptor 1
								0x00, 0x00, // index
								18, 0, // length
							];

							unsafe {
								core::ptr::write_volatile((0x50002000) as *mut [u8; 8], get_descriptor_packet);
							}

							otg_fs_host.hcchar1.modify(|_, w| w.chena().set_bit());

							while otg_fs_host.hcchar1.read().chena().bit() {
								//yield Some(UsbLoop::IgnoreThis);
							}
							
							otg_fs_host.hctsiz1.modify(|_, w| w
								.dpid().bits(2)
								.pktcnt().bits(1)
								.xfrsiz().bits(18)
							);
							otg_fs_host.hcchar1.modify(|_, w| w
								.dad().bits(1)
								.mcnt().bits(1)
								.epdir().set_bit()
								//.lsdev().set_bit() // TODO
								.epnum().bits(0) // 1 == in
								.eptyp().bits(0) // 0 == control
								.mpsiz().bits(64)
							);
							otg_fs_host.hcchar1.modify(|_, w| w.chena().set_bit());
							//core::ptr::write_volatile((0x50001000) as *mut [u8; 8], setup_packet);
					
							//while otg_fs_host.hcchar1.read().chena().bit() {
								writeln!(tx, "waiting for chena to become 0");
								while !otg_fs_global.gintsts.read().rxflvl().bit() {
									writeln!(tx, "waiting for rxflvl");
								}
								writeln!(tx, "gotcha");
								while otg_fs_global.gintsts.read().rxflvl().bit() { // FIXME duplicated code
									let rxstsp = otg_fs_global.grxstsp_host().read();
									writeln!(tx, "#{}: read ch={} dpid={} bcnt={} pktsts={} {}", frame_number, rxstsp.chnum().bits(), rxstsp.dpid().bits(), rxstsp.bcnt().bits(), rxstsp.pktsts().bits(),
										match rxstsp.pktsts().bits() {
											2 => "IN data packet received",
											3 => "IN transfer completed",
											5 => "Data toggle error",
											7 => "Channel halted",
											_ => "(unknown)"
										}
									).ok();
									for i in (0..20).step_by(4) {
										write!(tx, "{:08x} ", core::ptr::read_volatile((0x50002000) as *mut u32));
									}
									writeln!(tx, ".");
									// TODO do things https://github.com/libusbhost/libusbhost/blob/master/src/usbh_lld_stm32f4.c#L322
								}
							//}


							writeln!(tx, "done");
							
						}

						if val.pocchng().bit() {
							writeln!(tx, "overcurrent").ok();
						}

						if val.pcdet().bit() {
							writeln!(tx, "port connect detected (pcdet)").ok();
						}
				
						otg_fs_host.hprt.modify(|r,w| w.bits(r.bits() & !4)); // do not set PENA???
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
						trigger_pin.set_low();
						let haint = otg_fs_host.haint.read().bits();
						writeln!(tx, "#{} hcint (haint = {:08x})", frame_number, haint).ok();
						for i in 0..8 {
							if haint & (1 << i) != 0 {
								writeln!(tx, "hcint{} = {:08x}", i, otg_fs_host.hcintx(i).read().bits());
								otg_fs_host.hcintx(i).write(|w| w.bits(!0));
								writeln!(tx, "hcchar0 chena = {}", otg_fs_host.hcchar0.read().chena().bit());
							}
						}
					}

					//delay.delay_ms(1_u32);
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

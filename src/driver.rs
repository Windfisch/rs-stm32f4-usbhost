use core::future::Future;
use core::pin::Pin;
use crate::null_waker;
use core::cell::RefCell;

trait Driver {
	fn future(self: Pin<&mut Self>) -> Pin<&mut dyn Future<Output = ()>>;
}

type MidiDriverFuture = impl Future<Output = ()>;

static mut MIDI_DRIVER: Option<MidiDriver<4>> = None;

struct MidiDriver<const N: usize> {
	future: Option<MidiDriverFuture>,
	data: RefCell<MidiDriverData<N>>
}

struct MidiDriverData<const N: usize> {
	instances: [MidiDriverInstance; N],
}

impl<const N: usize> MidiDriverData<N> {
	fn new() -> MidiDriverData<N> {
		const blah: MidiDriverInstance = MidiDriverInstance { queue: heapless::Vec::new() };
		MidiDriverData {
			instances: [blah; N]
		}
	}
}

struct MidiDriverInstance {
	queue: heapless::Vec<[u8; 4], 16>
}


fn make_midi_driver_future<const N: usize>(foo: &RefCell<MidiDriverData<N>>) -> MidiDriverFuture {
	async fn foo() -> () {}
	foo()
}

impl<const N: usize> MidiDriver<N> {
	pub fn send(self: Pin<&'static mut Self>, device_id: u8, message: [u8; 4]) -> Result<(), [u8; 4]> {
		let mut data = self.data.borrow_mut();
		data.instances[device_id as usize].queue.push(message)
	}

	fn create_singleton() -> Pin<&'static mut MidiDriver<4>> {
		// FIXME critical section
		let driver: &mut _ = unsafe { &mut MIDI_DRIVER };

		if driver.is_some() {
			panic!();
		}

		*driver = Some(
			MidiDriver {
				future: None,
				data: RefCell::new( MidiDriverData::new() )
			}
		);

		driver.as_mut().unwrap().init();
		return Pin::static_mut(driver).as_pin_mut().unwrap();
	}

	fn init(&mut self) {
		self.future = Some(make_midi_driver_future(&self.data));
	}
}

impl<const N: usize> Driver for MidiDriver<N> {
	fn future(self: Pin<&mut Self>) -> Pin<&mut dyn Future<Output = ()>> {
		unsafe { Pin::new_unchecked(self.get_unchecked_mut().future.as_mut().unwrap()) }
	}
}

fn foo(driver: Pin<&mut dyn Driver>) {
	let waker = null_waker::create();
	let mut dummy_context = core::task::Context::from_waker(&waker);
	driver.future().poll(&mut dummy_context);
}

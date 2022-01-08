use core::future::Future;
use core::pin::Pin;
use crate::coroutine;
use core::cell::RefCell;
use crate::usb_host::UsbHost;

pub trait Driver {
	fn future(self: Pin<&mut Self>) -> Pin<&mut dyn Future<Output = ()>>;
}

type MidiDriverFuture<const N: usize> = impl Future<Output = ()>;

static mut MIDI_DRIVER: Option<MidiDriver<4>> = None;

pub struct MidiDriver<const N: usize> {
	future: Option<MidiDriverFuture<N>>,
	data: RefCell<MidiDriverData<N>>,
}

struct MidiDriverData<const N: usize> {
	instances: [MidiDriverInstance; N],
}

impl<const N: usize> MidiDriverData<N> {
	fn new() -> MidiDriverData<N> {
		const blah: MidiDriverInstance = MidiDriverInstance { 
			send_queue: heapless::Deque::new(),
			recv_queue: heapless::Deque::new(),
		};
		MidiDriverData {
			instances: [blah; N]
		}
	}
}

struct MidiDriverInstance {
	send_queue: heapless::Deque<[u8; 4], 16>,
	recv_queue: heapless::Deque<[u8; 4], 16>
}


async fn driver_future_func<const NN: usize>(host: &'static UsbHost, data: &'static RefCell<MidiDriverData<NN>>) -> () {
	data.borrow_mut();
}

// SAFETY: the resulting future will contain the `data` pointer. It must be ensured that
// `data` remains valid as long as the returned future lives.
unsafe fn make_midi_driver_future<const N: usize>(host: &'static UsbHost, data: *const RefCell<MidiDriverData<N>>) -> MidiDriverFuture<N> {
	driver_future_func(host, &*data)
}

impl<const N: usize> MidiDriver<N> {
	pub fn send(self: Pin<&'static mut Self>, device_id: u8, message: [u8; 4]) -> Result<(), [u8; 4]> {
		let mut data = self.data.borrow_mut();
		data.instances[device_id as usize].send_queue.push_back(message)
	}

	pub fn recv(self: Pin<&'static mut Self>, device_id: u8) -> Option<[u8;4]> {
		let mut data = self.data.borrow_mut();
		data.instances[device_id as usize].recv_queue.pop_front()
	}

	pub fn create_singleton(host: &'static UsbHost) -> Pin<&'static mut MidiDriver<4>> {
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

		// This is the point where we make driver self-referential: driver_ref.future
		// will contain a pointer to driver_ref.data. This pointer lives as long as
		// `driver` and so does driver_ref.data.
		// SAFETY: driver is never handed out directly, but only as Pin. Since future
		// is owned by driver itself, which also owns data, their lifetimes are equal.
		let driver_ref = driver.as_mut().unwrap();
		driver_ref.future = Some(unsafe { make_midi_driver_future(host, &driver_ref.data) });

		return Pin::static_mut(driver).as_pin_mut().unwrap();
	}
}

impl<const N: usize> Driver for MidiDriver<N> {
	fn future(self: Pin<&mut Self>) -> Pin<&mut dyn Future<Output = ()>> {
		unsafe { Pin::new_unchecked(self.get_unchecked_mut().future.as_mut().unwrap()) }
	}
}

fn foo(driver: Pin<&mut dyn Driver>) {
	let waker = coroutine::null_waker();
	let mut dummy_context = core::task::Context::from_waker(&waker);
	driver.future().poll(&mut dummy_context);
}

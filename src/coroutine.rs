// from https://blog.aloni.org/posts/a-stack-less-rust-coroutine-100-loc/

use core::task::{RawWaker, RawWakerVTable, Waker};

pub fn null_waker() -> Waker {
    // Safety: The waker points to a vtable with functions that do nothing. Doing
    // nothing is memory-safe.
    unsafe { Waker::from_raw(RAW_WAKER) }
}

const RAW_WAKER: RawWaker = RawWaker::new(core::ptr::null(), &VTABLE);
const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake_by_ref, drop);

unsafe fn clone(_: *const ()) -> RawWaker { RAW_WAKER }
unsafe fn wake(_: *const ()) { }
unsafe fn wake_by_ref(_: *const ()) { }
unsafe fn drop(_: *const ()) { }

pub fn poll(coroutine: Pin<&mut impl Future<Output=()>>) {
	let waker = null_waker();
	let mut dummy_context = core::task::Context::from_waker(&waker);
	let result = coroutine.poll(&mut dummy_context);
	match result {
		Poll::Ready(_) => unreachable!(),
		Poll::Pending => ()
	};
}

pub fn poll_dyn(coroutine: Pin<&mut dyn Future<Output=()>>) {
	let waker = null_waker();
	let mut dummy_context = core::task::Context::from_waker(&waker);
	let result = coroutine.poll(&mut dummy_context);
	match result {
		Poll::Ready(_) => unreachable!(),
		Poll::Pending => ()
	};
}


use core::cell::Cell;
use core::pin::Pin;
use core::future::Future;
use core::task::{Poll, Context};

/** A Future that needs to be `poll`ed exactly twice in order to get `Ready`.
  * Note that this future can not be used in the usual async executors such as tokio etc,
  * because it does not register a waker. Awaiting this future in such an executor will
  * block forever.
  */
pub struct YieldFuture {
	first: Cell<bool>
}

impl Future for YieldFuture {
	type Output = ();

    fn poll(self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<Self::Output> {
		if self.first.get() == true {
			self.first.set(false);
			return Poll::Pending;
		}
		else {
			return Poll::Ready(());
		}
	}
}

pub fn fyield() -> YieldFuture {
	YieldFuture { first: Cell::new(true) }
}

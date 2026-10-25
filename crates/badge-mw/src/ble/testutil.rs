//! Test-only executors: poll futures with a no-op waker, bounded so a hang
//! fails the test instead of spinning forever.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

/// Polls before a test executor gives up.
const MAX_POLLS: usize = 100_000;

/// Runs `f` to completion (it must not wait on anything external).
pub fn block_on<F: Future>(f: F) -> F::Output {
    let mut f = pin!(f);
    let mut cx = Context::from_waker(Waker::noop());
    for _ in 0..MAX_POLLS {
        if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
            return v;
        }
    }
    panic!("future did not complete");
}

/// Runs `a` and `b` concurrently until both complete.
pub fn join<A: Future, B: Future>(a: A, b: B) -> (A::Output, B::Output) {
    let mut a = pin!(a);
    let mut b = pin!(b);
    let mut ra = None;
    let mut rb = None;
    let mut cx = Context::from_waker(Waker::noop());
    for _ in 0..MAX_POLLS {
        if ra.is_none()
            && let Poll::Ready(v) = a.as_mut().poll(&mut cx)
        {
            ra = Some(v);
        }
        if rb.is_none()
            && let Poll::Ready(v) = b.as_mut().poll(&mut cx)
        {
            rb = Some(v);
        }
        if let (Some(_), Some(_)) = (&ra, &rb) {
            break;
        }
    }
    match (ra, rb) {
        (Some(a), Some(b)) => (a, b),
        _ => panic!("futures did not complete"),
    }
}

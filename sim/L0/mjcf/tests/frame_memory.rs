//! Expanding frames takes memory in proportion to the tree (ledger L296),
//! counted by a global allocator.
//!
//! A tree built in code alternating frame and body is the deepest a frame
//! can stand in (a document stops at 498 levels). Expanding one by copying
//! each frame's bodies, while the frame they were copied from was still
//! held, kept a copy of every subtree below each level alive at once: the
//! most bytes live grew with the square of the depth (2.5 GB of resident
//! memory at 4095 levels, measured).

// A counting allocator implements `GlobalAlloc`, which is an unsafe trait;
// each unsafe item forwards to the system allocator unchanged.
#![allow(unsafe_code)]
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use sim_mjcf::{MjcfModel, model_from_mjcf, parse_mjcf_str};

/// The bytes allocated and not yet freed, by every thread.
static LIVE: AtomicUsize = AtomicUsize::new(0);
/// The most `LIVE` has been since it was last reset.
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn grow(bytes: usize) {
    let live = LIVE.fetch_add(bytes, Ordering::SeqCst) + bytes;
    PEAK.fetch_max(live, Ordering::SeqCst);
}

fn shrink(bytes: usize) {
    LIVE.fetch_sub(bytes, Ordering::SeqCst);
}

/// The system allocator, counting the bytes live.
struct Counting;

// SAFETY: every method forwards its arguments to `System` unchanged, so it
// keeps `System`'s guarantees.
unsafe impl GlobalAlloc for Counting {
    // SAFETY: forwards to `System.alloc` with the caller's layout.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // SAFETY: the caller's layout, as `GlobalAlloc::alloc` requires.
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() {
            grow(layout.size());
        }
        ptr
    }

    // SAFETY: forwards to `System.dealloc` with the caller's pointer and layout.
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        shrink(layout.size());
        // SAFETY: `ptr` came from this allocator, that is from `System`, with `layout`.
        unsafe { System.dealloc(ptr, layout) }
    }

    // SAFETY: forwards to `System.realloc` with the caller's arguments.
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        // SAFETY: `ptr` came from `System` with `layout`, as the caller guarantees.
        let moved = unsafe { System.realloc(ptr, layout, new_size) };
        if !moved.is_null() {
            grow(new_size);
            shrink(layout.size());
        }
        moved
    }
}

#[global_allocator]
static ALLOCATOR: Counting = Counting;

/// A worldbody holding a body, which holds a frame holding a body, and so
/// on: `levels` bodies and frames in all, one of each per step down.
fn alternating(levels: usize) -> MjcfModel {
    let mut model =
        parse_mjcf_str("<mujoco><worldbody><frame><body name='b'/></frame></worldbody></mujoco>")
            .unwrap();
    let mut frame = model.worldbody.frames.pop().unwrap();
    let body = frame.bodies.pop().unwrap();
    let mut inner = body.clone();
    inner.name = "b0".into();
    for step in 1..levels / 2 {
        let mut holder = frame.clone();
        holder.bodies.push(inner);
        let mut outer = body.clone();
        outer.name = format!("b{step}");
        outer.frames.push(holder);
        inner = outer;
    }
    model.worldbody.children.push(inner);
    model
}

/// The most bytes live at once while `model_from_mjcf` loads a tree of
/// `levels`, above those live before.
fn peak_while_loading(levels: usize) -> usize {
    // A deep tree is built and dropped by recursion: on a thread with room.
    std::thread::Builder::new()
        .stack_size(256 << 20)
        .spawn(move || {
            let model = alternating(levels);
            let start = LIVE.load(Ordering::SeqCst);
            PEAK.store(start, Ordering::SeqCst);
            let built = model_from_mjcf(&model, None).unwrap();
            let peak = PEAK.load(Ordering::SeqCst) - start;
            assert_eq!(built.nbody, levels / 2 + 1);
            peak
        })
        .unwrap()
        .join()
        .unwrap()
}

#[test]
fn expanding_frames_takes_memory_in_proportion_to_the_tree() {
    // Four times the levels: about four times the bytes when they grow with
    // the tree, sixteen with its square; 8 lies between.
    let small = peak_while_loading(512);
    let large = peak_while_loading(2048);
    println!("the most bytes live: {small} at 512 levels, {large} at 2048");
    assert!(
        large < 8 * small,
        "the most bytes live: {small} at 512 levels, {large} at 2048"
    );
}

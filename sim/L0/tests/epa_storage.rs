//! The storage EPA takes (registry D-ARENA), counted by a global allocator.
//!
//! MuJoCo 3.5.0 holds EPA's polytope in static thread-local arrays for
//! `mjMAX_EPA_ITERATIONS` iterations and takes room for `6·ccd_iterations`
//! faces from its arena past that, stopping the step when they do not fit
//! (`engine_collision_gjk.c:2211-2215`, `:2304-2333`). sim-core keeps each
//! thread's polytope between calls and grows it only as EPA fills it.

// A counting allocator implements `GlobalAlloc`, which is an unsafe trait;
// each unsafe item forwards to the system allocator unchanged.
#![allow(unsafe_code)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use sim_mjcf::load_model;

thread_local! {
    /// The bytes this thread has asked for.
    static BYTES: Cell<usize> = const { Cell::new(0) };
}

fn count(bytes: usize) {
    // A thread being torn down no longer has its counter; nothing to count.
    let _ = BYTES.try_with(|b| b.set(b.get() + bytes));
}

/// The system allocator, counting each thread's requests.
struct Counting;

// SAFETY: every method forwards its arguments to `System` unchanged, so it
// keeps `System`'s guarantees.
unsafe impl GlobalAlloc for Counting {
    // SAFETY: forwards to `System.alloc` with the caller's layout.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count(layout.size());
        // SAFETY: the caller's layout, as `GlobalAlloc::alloc` requires.
        unsafe { System.alloc(layout) }
    }

    // SAFETY: forwards to `System.dealloc` with the caller's pointer and layout.
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` came from this allocator, that is from `System`, with `layout`.
        unsafe { System.dealloc(ptr, layout) }
    }

    // SAFETY: forwards to `System.realloc` with the caller's arguments.
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count(new_size);
        // SAFETY: `ptr` came from `System` with `layout`, as the caller guarantees.
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOCATOR: Counting = Counting;

/// On a thread of its own, so that EPA's storage starts empty: after `warm`
/// calls of `forward` on `xml`, the bytes the thread asks for during one
/// more, and the one contact it makes.
fn forward_on_a_new_thread(xml: String, warm: usize) -> (usize, [u64; 4]) {
    std::thread::spawn(move || {
        let model = load_model(&xml).expect("the scene loads");
        let mut data = model.make_data();
        for _ in 0..warm {
            data.forward(&model).expect("forward");
        }
        let before = BYTES.with(Cell::get);
        data.forward(&model).expect("forward");
        let bytes = BYTES.with(Cell::get) - before;
        assert_eq!(data.ncon, 1, "{xml}");
        let c = &data.contacts[0];
        (
            bytes,
            [c.depth, c.pos.x, c.pos.y, c.pos.z].map(f64::to_bits),
        )
    })
    .join()
    .expect("the thread finishes")
}

/// An ellipsoid sunk 0.3 into a box, so that EPA runs, under
/// `ccd_iterations`.
fn ellipsoid_in_a_box(ccd_iterations: u32) -> String {
    format!(
        r#"<mujoco><option ccd_iterations="{ccd_iterations}"/><worldbody>
             <geom type="box" size="0.5 0.5 0.5"/>
             <body pos="0 0 0.3"><freejoint/><geom type="ellipsoid" size="0.2 0.15 0.1"/></body>
           </worldbody></mujoco>"#
    )
}

/// A sphere of radius 0.2 on an ellipsoid whose top is at 0.3, its centre at
/// `z`. GJK takes the sphere as its centre (`mjc_ccd`'s point support): from
/// a centre outside the ellipsoid it inflates the distance by the radius and
/// EPA does not run; from one inside, EPA runs.
fn sphere_on_an_ellipsoid(z: f64) -> String {
    format!(
        r#"<mujoco><worldbody>
             <geom type="ellipsoid" size="0.5 0.4 0.3"/>
             <body pos="0 0 {z}"><freejoint/><geom type="sphere" size="0.2"/></body>
           </worldbody></mujoco>"#
    )
}

/// EPA's storage grows with what it holds, not with `ccd_iterations`: a
/// thread's first call under a million iterations asks for the bytes one
/// under 35 does and makes the same contact, bit for bit (EPA ends long
/// before either limit). MuJoCo's arena stops that step
/// (`mj_stackAlloc: out of memory`; 100,000 iterations stop it on this
/// scene, as `gen_collision_reference.py`'s `mujoco_arena_limits` asserts).
#[test]
fn epa_storage_grows_with_use_not_with_ccd_iterations() {
    assert_eq!(
        forward_on_a_new_thread(ellipsoid_in_a_box(1_000_000), 0),
        forward_on_a_new_thread(ellipsoid_in_a_box(35), 0)
    );
}

/// A thread keeps EPA's storage between calls: once it has held a call's
/// polytope, a `forward` in which EPA runs (the sphere's centre 0.1 inside
/// the ellipsoid) asks for the bytes of one in which it does not (0.15
/// outside).
#[test]
fn a_thread_keeps_epa_s_storage_between_calls() {
    assert_eq!(
        forward_on_a_new_thread(sphere_on_an_ellipsoid(0.2), 3).0,
        forward_on_a_new_thread(sphere_on_an_ellipsoid(0.45), 3).0
    );
}

//! SDF Physics 04 — Socket
//!
//! Condyle-in-socket: a convex knob rotates inside a concave housing.
//! The socket walls physically constrain the range of motion — no joint
//! limits needed. This is the goal: SDF === collision for articulated
//! bio-inspired mechanisms.
//!
//! Pass criteria:
//! - Condyle rotates freely within the socket void
//! - Socket walls stop the condyle at the physical limit
//! - Range of motion matches the printed geometry exactly
//! - No vibration, tunneling, or explosion at contact
//!
//! Depends on: 03-hinge working.
//! Blocked by: multi-contact `sdf_sdf_contact` for concave constraints.
//!
//! Run with: `cargo run -p example-sdf-04-socket --release`

fn main() {
    eprintln!("BLOCKED: needs 03-hinge first — see examples/EXAMPLES.md");
}

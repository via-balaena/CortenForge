//! SDF Physics 16 — Socket (Full Condyle Articulation)
//!
//! Condyle-in-socket: a convex knob rotates inside a concave housing. The
//! socket walls physically constrain the range of motion — no joint limits
//! needed. This is the goal: SDF === collision for articulated bio-inspired
//! mechanisms.
//!
//! Uses the exact geometry from finger-design: socket wall 0.6 mm,
//! SDF resolution 0.5 mm (proven by step 02).
//!
//! Requires multi-contact `sdf_sdf_contact` — a single contact point cannot
//! constrain a concave socket from all directions simultaneously.
//!
//! Pass criteria:
//! - Condyle rotates freely within the socket void
//! - Socket walls stop the condyle at the physical limit
//! - Range of motion matches the printed geometry exactly
//! - No vibration, tunneling, or explosion at contact
//! - Multiple contact points along the socket wall
//!
//! New concept: concave multi-contact SDF-SDF (full socket)
//! Depends on: 13-concave-stop
//! Blocked by: multi-contact `sdf_sdf_contact` (if step 13 reveals it's needed)
//!
//! Run with: `cargo run -p example-sdf-16-socket --release`

fn main() {
    eprintln!("BLOCKED: needs multi-contact sdf_sdf_contact — see examples/EXAMPLES.md");
}

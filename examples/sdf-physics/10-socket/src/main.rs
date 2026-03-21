//! SDF Physics 10 — Socket (Concave Multi-Contact)
//!
//! Condyle-in-socket: a convex knob rotates inside a concave housing. The
//! socket walls physically constrain the range of motion — no joint limits
//! needed. This is the goal: SDF === collision for articulated bio-inspired
//! mechanisms.
//!
//! Requires multi-contact `sdf_sdf_contact` — a single contact point cannot
//! constrain a concave socket from all directions simultaneously.
//!
//! Pass criteria:
//! - Condyle rotates freely within the socket void
//! - Socket walls stop the condyle at the physical limit
//! - Range of motion matches the printed geometry exactly
//! - No vibration, tunneling, or explosion at contact
//! - Multiple contact points visible along the socket wall
//!
//! New concept: concave constraint, multi-contact SDF-SDF
//! Depends on: 09-hinge-stop (parent-child SDF collision works for convex)
//! Blocked by: multi-contact `sdf_sdf_contact` (not yet implemented)
//!
//! Run with: `cargo run -p example-sdf-10-socket --release`

fn main() {
    eprintln!("BLOCKED: needs multi-contact sdf_sdf_contact — see examples/EXAMPLES.md");
}

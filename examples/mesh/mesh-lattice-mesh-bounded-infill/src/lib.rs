//! Library side of `example-mesh-mesh-lattice-mesh-bounded-infill` —
//! exposes the shared 50 mm cube fixture so both the default binary
//! (`src/main.rs`, fixture-side anchors that stay green across the
//! F6 sub-arc) and the audit-trail binary
//! (`src/bin/pre_fix_check.rs`, v0.7 baseline witnesses for gaps
//! a/b/c/d/e — deleted at `§6.2 #29` once post-fix anchors land)
//! consume the same hand-authored cube without duplicating its
//! topology.

pub mod fixture;

pub use fixture::{
    FACES, OUTWARD_NORMALS, SIDE, TIGHT_TOL, VERTS, VOLUME_TOL, WINDING_COSINE_THRESHOLD,
    cube_50mm, cube_at_origin, verify_fixture,
};

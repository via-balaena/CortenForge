//! Library side of `example-mesh-mesh-lattice-mesh-bounded-infill` —
//! exposes the shared 50 mm cube fixture used by the binary
//! (`src/main.rs`).

pub mod fixture;

pub use fixture::{
    FACES, OUTWARD_NORMALS, SIDE, TIGHT_TOL, VERTS, VOLUME_TOL, WINDING_COSINE_THRESHOLD,
    cube_50mm, cube_at_origin, verify_fixture,
};

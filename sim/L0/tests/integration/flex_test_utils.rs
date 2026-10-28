//! The vertex bodies of a `<flex body=...>`, for flex documents a test builds
//! at run time.
//!
//! MuJoCo 3.5.0's `<flex>` names existing bodies as its vertices
//! (`mjCFlex::Compile`, `user_mesh.cc:4103-4213`), and sim-mjcf takes each
//! vertex body jointless or with three slides along x, y and z. The in-tree
//! flex documents were rewritten into that form (Rigid-loading L10): one body
//! per vertex, at the end of `<worldbody>`, with the mass sim-mjcf had lumped
//! onto the vertex where a test reads the dynamics (`flex_data_parity`, which
//! compares positions and boxes only, gives each vertex 1 g).

/// The bodies of a flex `flex`'s vertices at `positions` (each three numbers
/// as written), named `<flex>_v<i>`, each with three slides along x, y and z
/// (none for a vertex in `pinned`), the slides' `damping` when it is given,
/// and a point mass `masses[i]` (printed to round-trip); and the names, for
/// the flex's `body`.
pub fn vertex_bodies(
    flex: &str,
    positions: &[String],
    masses: &[f64],
    pinned: &[usize],
    damping: Option<&str>,
) -> (String, String) {
    assert_eq!(positions.len(), masses.len(), "one mass per vertex");
    let damping = damping.map_or(String::new(), |d| format!(r#" damping="{d}""#));
    let mut bodies = String::new();
    let mut names = Vec::with_capacity(positions.len());
    for (i, (pos, mass)) in positions.iter().zip(masses).enumerate() {
        let name = format!("{flex}_v{i}");
        let slides = if pinned.contains(&i) {
            String::new()
        } else {
            ["1 0 0", "0 1 0", "0 0 1"]
                .map(|axis| format!(r#"<joint type="slide" axis="{axis}"{damping}/>"#))
                .concat()
        };
        bodies += &format!(
            r#"<body name="{name}" pos="{pos}">{slides}<inertial pos="0 0 0" mass="{mass:?}" diaginertia="1e-15 1e-15 1e-15"/></body>"#
        );
        names.push(name);
    }
    (bodies, names.join(" "))
}

/// Each vertex's three numbers from a flat list of them, as written.
pub fn triples(coordinates: &str) -> Vec<String> {
    let numbers: Vec<&str> = coordinates.split_whitespace().collect();
    assert_eq!(numbers.len() % 3, 0, "three numbers per vertex");
    numbers.chunks(3).map(|c| c.join(" ")).collect()
}

/// `n` vertices sharing a total mass `total`, as sim-mjcf lumped a flex's
/// `mass` (`total / n` each).
#[allow(clippy::cast_precision_loss)] // a vertex count, far below 2^52
pub fn uniform(total: f64, n: usize) -> Vec<f64> {
    vec![total / n as f64; n]
}

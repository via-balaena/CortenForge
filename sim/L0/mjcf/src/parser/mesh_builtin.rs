//! MuJoCo 3.5.0's built-in meshes, `<mesh builtin="…" params="…"/>`.
//!
//! MuJoCo's reader hands a mesh's `builtin` and `params` to `mjs_makeMesh`
//! (`mjXReader::OneMesh`, `xml_native_reader.cc:1621-1634`), which checks the
//! parameters (`user_api.cc:580-731`) and has one of seven generators
//! (`mjCMesh::Make*`, `user_mesh.cc:2072-2560`) write the mesh's vertices,
//! as floats, and all but the cone's and the wedge's faces; the plate's also
//! makes the mesh a shell. The compiler then sees the mesh as one written
//! with `vertex` and `face`. [`make_mesh`] ports the checks and the
//! generators, each sum, product and rounding to a float in MuJoCo's order,
//! with the platform's `cos`, `sin` and `pow`, as MuJoCo calls them. The
//! normals the wedge and the plate also write are not made: this crate keeps
//! no mesh's normals.

use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::types::MeshInertia;

/// A mesh's `builtin` keyword (`meshbuiltin_map`,
/// `xml_native_reader.cc:877-885`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshBuiltin {
    /// `none`, which `mjs_makeMesh` refuses ("Unsupported mesh type").
    None,
    /// `sphere`: an icosahedron, subdivided.
    Sphere,
    /// `hemisphere`: a quad-projected half sphere with a flat cap.
    Hemisphere,
    /// `cone`: a discrete cone or frustum; vertices only.
    Cone,
    /// `supersphere`: a superellipsoid.
    Supersphere,
    /// `supertorus`: a supertoroid.
    Supertorus,
    /// `wedge`: points on a spherical wedge; vertices only.
    Wedge,
    /// `plate`: a flat grid of points with its faces, a shell.
    Plate,
}

impl MeshBuiltin {
    /// The keyword's kind, as `meshbuiltin_map` names it.
    pub fn from_keyword(s: &str) -> Option<Self> {
        match s {
            "none" => Some(Self::None),
            "sphere" => Some(Self::Sphere),
            "hemisphere" => Some(Self::Hemisphere),
            "cone" => Some(Self::Cone),
            "supertorus" => Some(Self::Supertorus),
            "supersphere" => Some(Self::Supersphere),
            "wedge" => Some(Self::Wedge),
            "plate" => Some(Self::Plate),
            _ => None,
        }
    }
}

/// What a generator writes into a mesh's specification.
#[derive(Debug, Clone, PartialEq)]
pub struct BuiltinMesh {
    /// Its vertices, three floats each (`uservert`).
    pub vertex: Vec<f32>,
    /// Its faces, three vertex ids each (`userface`), where the generator
    /// writes them: the cone's and the wedge's do not, so a mesh's written
    /// `face` stays.
    pub face: Option<Vec<u32>>,
    /// The inertia mode it sets, whatever was written: the plate's is
    /// [`MeshInertia::Shell`] (`MakeRect`, `user_mesh.cc:2501`).
    pub inertia: Option<MeshInertia>,
}

/// The mesh `mjs_makeMesh` makes of `builtin` and `params`
/// (`user_api.cc:580-731`), or its refusal in MuJoCo's words.
///
/// Each kind takes an exact count of parameters, some of which are cast to
/// integers ([`as_int`]) and checked as MuJoCo checks them. Two inputs that
/// MuJoCo goes on with are refused here (`D-MESH-BUILTIN-UNDEFINED`): a cone
/// of fewer than 0 edges, which MuJoCo builds a vector of negative size for
/// (measured: the error "vector", or at a radius of 0 a segmentation fault),
/// and a mesh whose counts overflow the `int` arithmetic MuJoCo sizes it with
/// (measured: "vector", "std::bad_alloc", or gigabytes allocated).
pub fn make_mesh(builtin: MeshBuiltin, params: &[f64]) -> Result<BuiltinMesh, String> {
    match builtin {
        MeshBuiltin::Hemisphere => {
            let [res] = take(params, "Hemisphere mesh type requires 1 parameter")?;
            let res = as_int(res);
            if res < 0 {
                return Err("Hemisphere resolution cannot be negative".into());
            }
            if res > 10 {
                return Err("Hemisphere resolution cannot be greater than 10".into());
            }
            Ok(hemisphere(res))
        }
        MeshBuiltin::Sphere => {
            let [subdiv] = take(params, "Sphere mesh type requires 1 parameter")?;
            let subdiv = as_int(subdiv);
            if subdiv < 0 {
                return Err("Sphere subdivision cannot be negative".into());
            }
            if subdiv > 4 {
                return Err("Sphere subdivision cannot be greater than 4".into());
            }
            Ok(sphere(subdiv))
        }
        MeshBuiltin::Supersphere => {
            let [res, e, n] = take(params, "Supersphere mesh type requires 3 parameters")?;
            let res = as_int(res);
            if res < 3 {
                return Err("Supersphere resolution must be greater than 2".into());
            }
            if e < 0.0 {
                return Err("Supersphere 'e' cannot be negative".into());
            }
            if n < 0.0 {
                return Err("Supersphere 'n' cannot be negative".into());
            }
            sized(supersphere_sizes(res), "Supersphere")?;
            Ok(supersphere(res, e, n))
        }
        MeshBuiltin::Supertorus => {
            let [res, radius, s, t] = take(params, "Supertorus mesh type requires 4 parameters")?;
            let res = as_int(res);
            if res < 3 {
                return Err("Supertorus resolution must be greater than 3".into());
            }
            if radius <= 0.0 || radius > 1.0 {
                return Err("Supertorus radius must be in (0, 1]".into());
            }
            if s <= 0.0 {
                return Err("Supertorus 's' must be greater than 0".into());
            }
            if t <= 0.0 {
                return Err("Supertorus 't' must be greater than 0".into());
            }
            sized(supertorus_sizes(res), "Supertorus")?;
            Ok(supertorus(res, radius, s, t))
        }
        MeshBuiltin::Wedge => {
            let [r0, r1, fov0, fov1, gamma] =
                take(params, "Wedge builtin mesh types require 5 parameters")?;
            let resolution = [as_int(r0), as_int(r1)];
            if fov0 <= 0.0 || fov0 > 180.0 {
                return Err("fov[0] must be a float between (0, 180] degrees".into());
            }
            if fov1 <= 0.0 || fov1 > 90.0 {
                return Err("`fov[1]` must be a float between (0, 90] degrees".into());
            }
            if resolution[0] <= 0 || resolution[1] <= 0 {
                return Err("Horizontal and vertical resolutions must be positive".into());
            }
            if !(0.0..=1.0).contains(&gamma) {
                return Err("`gamma` must be a nonnegative float between [0, 1]".into());
            }
            sized(grid_sizes(resolution), "Wedge")?;
            Ok(wedge(resolution, [fov0, fov1], gamma))
        }
        MeshBuiltin::Plate => {
            let [r0, r1] = take(params, "Plate builtin mesh type requires 2 parameters")?;
            let resolution = [as_int(r0), as_int(r1)];
            if resolution[0] <= 0 || resolution[1] <= 0 {
                return Err("Horizontal and vertical resolutions must be positive".into());
            }
            sized(grid_sizes(resolution), "Plate")?;
            Ok(plate(resolution))
        }
        MeshBuiltin::Cone => {
            let [nedge, radius] = take(params, "Cone mesh type requires 2 parameters")?;
            let nedge = as_int(nedge);
            if nedge < 0 {
                return Err("Cone edge count cannot be negative".into());
            }
            sized(cone_sizes(nedge, radius), "Cone")?;
            Ok(cone(nedge, radius))
        }
        MeshBuiltin::None => Err("Unsupported mesh type".into()),
    }
}

/// The parameters, exactly `N` of them, or `refusal`.
fn take<const N: usize>(params: &[f64], refusal: &str) -> Result<[f64; N], String> {
    <[f64; N]>::try_from(params).map_err(|_| refusal.to_owned())
}

/// A parameter as `mjs_makeMesh`'s `static_cast<int>` makes it. C++ leaves a
/// value outside `int`'s range undefined; on arm64, where this crate's
/// goldens are measured, it truncates toward zero, saturating at `int`'s
/// bounds, and makes a NaN 0, as Rust's `as` does (measured: `params="1e10"`
/// is refused for a sphere as greater than 4).
#[allow(clippy::cast_possible_truncation)] // the cut is `static_cast<int>`'s
fn as_int(x: f64) -> i32 {
    x as i32
}

/// Refused where MuJoCo's `int` arithmetic sizing the mesh overflows
/// (`sizes` is `None`).
fn sized(sizes: Option<()>, kind: &str) -> Result<(), String> {
    sizes.ok_or_else(|| format!("{kind} mesh is too large"))
}

/// `MakeSupersphere`'s sizes, each `int` expression it computes them by
/// (`user_mesh.cc:2300-2305`), `None` where one overflows.
fn supersphere_sizes(res: i32) -> Option<()> {
    let nvert = (res - 1).checked_mul(res)?.checked_add(2)?;
    let nface = 2_i32.checked_mul(res)?.checked_mul(res - 1)?;
    3_i32.checked_mul(nvert)?;
    3_i32.checked_mul(nface)?;
    Some(())
}

/// `MakeSupertorus`'s sizes (`user_mesh.cc:2415-2418`), as
/// [`supersphere_sizes`].
fn supertorus_sizes(res: i32) -> Option<()> {
    let nvert = res.checked_mul(res)?;
    let nface = res.checked_mul(res)?.checked_mul(2)?;
    3_i32.checked_mul(nvert)?;
    3_i32.checked_mul(nface)?;
    Some(())
}

/// The wedge's and the plate's sizes (`MakeWedge`, `user_mesh.cc:2467-2471`;
/// `MakeRect`, `:2494-2500`): the bin edges, three floats per vertex, nine
/// per normal and six ids per cell, as [`supersphere_sizes`].
fn grid_sizes([r0, r1]: [i32; 2]) -> Option<()> {
    r0.checked_add(1)?;
    r1.checked_add(1)?;
    3_i32.checked_mul(r0)?.checked_mul(r1)?;
    9_i32.checked_mul(r0)?.checked_mul(r1)?;
    6_i32.checked_mul(r0 - 1)?.checked_mul(r1 - 1)?;
    Some(())
}

/// `MakeCone`'s sizes (`user_mesh.cc:2538-2557`), as [`supersphere_sizes`].
fn cone_sizes(nedge: i32, radius: f64) -> Option<()> {
    if radius > 0.0 {
        3_i32.checked_mul(nedge.checked_add(nedge)?)?;
    } else {
        3_i32.checked_mul(nedge.checked_add(1)?)?;
    }
    Some(())
}

/// A count or id the checks have made non-negative, as an index.
#[allow(clippy::cast_sign_loss)] // [`make_mesh`] refuses a negative count
fn index(n: i32) -> usize {
    n as usize
}

/// A vertex id, as a face holds it.
#[allow(clippy::cast_possible_truncation)] // the size check keeps every id below `i32::MAX`
fn id(n: usize) -> u32 {
    n as u32
}

/// `mjCMesh::MakeHemisphere` with faces and a cap
/// (`user_mesh.cc:2072-2189`): the north pole; rows of points from it to the
/// equator, each row's four arcs interpolated in double and pushed out to
/// the unit sphere; the cap's centre last.
#[allow(clippy::cast_possible_truncation)] // each cast is the store into a float
fn hemisphere(res: i32) -> BuiltinMesh {
    const NORTH: [f64; 3] = [0.0, 0.0, 1.0];
    const EQUATOR: [[f64; 3]; 4] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ];
    let r = index(res);
    let nvert = 1 + 2 * (r + 1) * (r + 2) + 1;
    let mut vert = vec![0.0_f32; 3 * nvert];
    vert[2] = 1.0;

    let mut v = 1;
    for row in 0..=res {
        for side in 0..4 {
            let factor = f64::from(row + 1) / f64::from(res + 1);
            let start: [f64; 3] =
                std::array::from_fn(|i| NORTH[i] + factor * (EQUATOR[side][i] - NORTH[i]));
            let end: [f64; 3] = std::array::from_fn(|i| {
                NORTH[i] + factor * (EQUATOR[(side + 1) % 4][i] - NORTH[i])
            });
            let delta: [f64; 3] = std::array::from_fn(|i| (end[i] - start[i]) / f64::from(row + 1));
            for i in 0..=row {
                let p: [f64; 3] = std::array::from_fn(|j| start[j] + f64::from(i) * delta[j]);
                let norm = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
                for j in 0..3 {
                    vert[3 * v + j] = (p[j] / norm) as f32;
                }
                v += 1;
            }
        }
    }

    let mut face: Vec<u32> = vec![0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 1];
    for row in 0..r {
        let start_curr = 2 * row * (row + 1) + 1;
        let count_curr = 4 * (row + 1);
        let start_next = start_curr + count_curr;
        let count_next = 4 * (row + 2);
        for side in 0..4 {
            for i in 0..row + 2 {
                let v_curr = i + (row + 1) * side;
                let v_next = i + (row + 2) * side;
                face.extend(
                    [
                        start_curr + v_curr % count_curr,
                        start_next + v_next % count_next,
                        start_next + (v_next + 1) % count_next,
                    ]
                    .map(id),
                );
                if i < row + 1 {
                    face.extend(
                        [
                            start_curr + v_curr % count_curr,
                            start_next + (v_next + 1) % count_next,
                            start_curr + (v_curr + 1) % count_curr,
                        ]
                        .map(id),
                    );
                }
            }
        }
    }
    let start = 2 * r * (r + 1) + 1;
    let count = 4 * (r + 1);
    for i in 0..count {
        face.extend([start + i, nvert - 1, start + (i + 1) % count].map(id));
    }
    BuiltinMesh {
        vertex: vert,
        face: Some(face),
        inertia: None,
    }
}

/// `mjCMesh::MakeSphere` with faces (`user_mesh.cc:2194-2293`): an
/// icosahedron on the unit sphere, each triangle split in four `subdiv`
/// times, each edge's midpoint made once per subdivision and pushed out to
/// the sphere; every value a float but the first normalisation's divisor.
#[allow(clippy::cast_possible_truncation)] // each cast is the store into a float
fn sphere(subdiv: i32) -> BuiltinMesh {
    // `const float phi = (1.0 + std::sqrt(5.0)) / 2.0`, the sum and the
    // halving MuJoCo's, not a midpoint function's.
    #[allow(clippy::manual_midpoint)]
    let phi = ((1.0 + 5.0_f64.sqrt()) / 2.0) as f32;
    #[rustfmt::skip]
    let mut vert: Vec<f32> = vec![
        -1.0,  phi, 0.0,    1.0,  phi, 0.0,   -1.0, -phi, 0.0,    1.0, -phi, 0.0,
         0.0, -1.0, phi,    0.0,  1.0, phi,    0.0, -1.0, -phi,   0.0,  1.0, -phi,
         phi,  0.0, -1.0,   phi,  0.0, 1.0,   -phi,  0.0, -1.0,  -phi,  0.0, 1.0,
    ];
    // `std::sqrt(1.0 + phi * phi)`: the square a float's, the sum a double.
    let norm = (1.0 + f64::from(phi * phi)).sqrt();
    for x in &mut vert {
        *x = (f64::from(*x) / norm) as f32;
    }
    #[rustfmt::skip]
    let mut face: Vec<u32> = vec![
        0, 11, 5,   0, 5, 1,    0, 1, 7,    0, 7, 10,   0, 10, 11,
        1, 5, 9,    5, 11, 4,   11, 10, 2,  10, 7, 6,   7, 1, 8,
        3, 9, 4,    3, 4, 2,    3, 2, 6,    3, 6, 8,    3, 8, 9,
        4, 9, 5,    2, 4, 11,   6, 2, 10,   8, 6, 7,    9, 8, 1,
    ];
    for _ in 0..subdiv {
        let mut cache = BTreeMap::new();
        let mut next = Vec::with_capacity(face.len() * 4);
        for tri in face.chunks_exact(3) {
            let [v1, v2, v3] = [tri[0], tri[1], tri[2]];
            let m12 = midpoint(&mut vert, &mut cache, v1, v2);
            let m23 = midpoint(&mut vert, &mut cache, v2, v3);
            let m31 = midpoint(&mut vert, &mut cache, v3, v1);
            next.extend([v1, m12, m31, v2, m23, m12, v3, m31, m23, m12, m23, m31]);
        }
        face = next;
    }
    BuiltinMesh {
        vertex: vert,
        face: Some(face),
        inertia: None,
    }
}

/// `MakeSphere`'s `get_midpoint` (`user_mesh.cc:2230-2265`): the vertex
/// between `a` and `b`, made the first time the edge is met in this
/// subdivision (`cache`, keyed by the edge's ids in order) and appended.
fn midpoint(vert: &mut Vec<f32>, cache: &mut BTreeMap<(u32, u32), u32>, a: u32, b: u32) -> u32 {
    let key = (a.min(b), a.max(b));
    if let Some(&made) = cache.get(&key) {
        return made;
    }
    let (p, q) = (3 * a as usize, 3 * b as usize);
    // `(v1[0] + v2[0]) / 2.0f`: a float sum halved, MuJoCo's, not a midpoint
    // function's.
    #[allow(clippy::manual_midpoint)]
    let mid: [f32; 3] = std::array::from_fn(|k| (vert[p + k] + vert[q + k]) / 2.0);
    let norm = (mid[0] * mid[0] + mid[1] * mid[1] + mid[2] * mid[2]).sqrt();
    let made = id(vert.len() / 3);
    vert.extend(mid.map(|x| x / norm));
    cache.insert(key, made);
    made
}

/// `MakeSupersphere`'s and `MakeSupertorus`'s `aux_c`
/// (`user_mesh.cc:146-148`): `cos(omega)` raised to `m` in magnitude, its
/// sign kept.
fn aux_c(omega: f64, m: f64) -> f64 {
    omega.cos().abs().powf(m).copysign(omega.cos())
}

/// `aux_s` (`user_mesh.cc:149-151`): as [`aux_c`], of `sin(omega)`.
fn aux_s(omega: f64, m: f64) -> f64 {
    omega.sin().abs().powf(m).copysign(omega.sin())
}

/// `mjCMesh::MakeSupersphere` (`user_mesh.cc:2298-2355`): the south pole,
/// `res - 1` rings of `res` points from south to north, the north pole; a
/// fan of faces at each pole and two faces per ring cell.
#[allow(clippy::cast_possible_truncation)] // each cast is the store into a float
#[allow(clippy::many_single_char_names)] // MuJoCo's names, to read beside `user_mesh.cc`
fn supersphere(res: i32, e: f64, n: f64) -> BuiltinMesh {
    let r = index(res);
    let nvert = (r - 1) * r + 2;
    let mut vert: Vec<f32> = Vec::with_capacity(3 * nvert);
    vert.extend([0.0, 0.0, -1.0]);
    for i in 1..res {
        let v = -PI / 2.0 + f64::from(i) * PI / f64::from(res);
        for j in 0..res {
            let u = -PI + f64::from(j * 2) * PI / f64::from(res);
            vert.extend([
                (aux_c(v, n) * aux_c(u, e)) as f32,
                (aux_c(v, n) * aux_s(u, e)) as f32,
                aux_s(v, n) as f32,
            ]);
        }
    }
    vert.extend([0.0, 0.0, 1.0]);

    let mut face: Vec<u32> = Vec::with_capacity(6 * r * (r - 1));
    for j in 0..r {
        face.extend([0, 1 + (j + 1) % r, 1 + j].map(id));
    }
    for i in 0..r - 2 {
        for j in 0..r {
            let v1 = 1 + i * r + j;
            let v2 = 1 + i * r + (j + 1) % r;
            let v4 = 1 + (i + 1) * r + j;
            let v3 = 1 + (i + 1) * r + (j + 1) % r;
            face.extend([v1, v2, v4, v2, v3, v4].map(id));
        }
    }
    let north = nvert - 1;
    let last_ring = 1 + (r - 2) * r;
    for j in 0..r {
        face.extend([last_ring + j, last_ring + (j + 1) % r, north].map(id));
    }
    BuiltinMesh {
        vertex: vert,
        face: Some(face),
        inertia: None,
    }
}

/// `mjCMesh::MakeSupertorus` (`user_mesh.cc:2413-2461`): `res × res`
/// points, `u` around the axis and `v` around the tube, two faces per cell.
#[allow(clippy::cast_possible_truncation)] // each cast is the store into a float
#[allow(clippy::many_single_char_names)] // MuJoCo's names, to read beside `user_mesh.cc`
fn supertorus(res: i32, radius: f64, s: f64, t: f64) -> BuiltinMesh {
    let r = index(res);
    let mut vert = vec![0.0_f32; 3 * r * r];
    for i in 0..res {
        for j in 0..res {
            let u = 2.0 * PI * f64::from(i) / f64::from(res);
            let v = 2.0 * PI * f64::from(j) / f64::from(res);
            let at = 3 * (index(i) * r + index(j));
            vert[at] = ((1.0 + radius * aux_c(v, s)) * aux_c(u, t)) as f32;
            vert[at + 1] = ((1.0 + radius * aux_c(v, s)) * aux_s(u, t)) as f32;
            vert[at + 2] = (radius * aux_s(v, s)) as f32;
        }
    }
    let mut face: Vec<u32> = Vec::with_capacity(6 * r * r);
    for i in 0..r {
        for j in 0..r {
            let (i_next, j_next) = ((i + 1) % r, (j + 1) % r);
            let v1 = i * r + j;
            let v2 = i_next * r + j;
            let v3 = i_next * r + j_next;
            let v4 = i * r + j_next;
            face.extend([v1, v2, v4, v2, v3, v4].map(id));
        }
    }
    BuiltinMesh {
        vertex: vert,
        face: Some(face),
        inertia: None,
    }
}

/// `LinSpace` (`user_mesh.cc:94-101`): `n` numbers from `lower`, each the
/// last plus the step.
fn lin_space(lower: f64, upper: f64, n: i32) -> Vec<f64> {
    let increment = if n > 1 {
        (upper - lower) / f64::from(n - 1)
    } else {
        0.0
    };
    let mut x = lower;
    (0..n)
        .map(|_| {
            let at = x;
            x += increment;
            at
        })
        .collect()
}

/// `Fovea` (`user_mesh.cc:84-91`): a gamma's blend of `x` and `x⁵`.
fn fovea(x: f64, gamma: f64) -> f64 {
    if gamma == 0.0 {
        return x;
    }
    // `mjMAX(0, mjMIN(1, gamma))`, as the macros compare.
    let low = if 1.0 < gamma { 1.0 } else { gamma };
    let g = if 0.0 > low { 0.0 } else { low };
    g * x.powf(5.0) + (1.0 - g) * x
}

/// `BinEdges` (`user_mesh.cc:104-121`): the unit edges of `size` bins,
/// deformed by [`fovea`] and scaled to the field of view in radians.
fn bin_edges(size: i32, fov: f64, gamma: f64) -> Vec<f64> {
    let scale = fov * PI / 180.0;
    lin_space(-1.0, 1.0, size + 1)
        .into_iter()
        .map(|x| scale * fovea(x, gamma))
        .collect()
}

/// `mjCMesh::MakeWedge` (`user_mesh.cc:2466-2488`): a point at each bin's
/// centre in azimuth and elevation, on the unit sphere
/// (`SphericalToCartesian`, `:124-129`); no faces.
#[allow(clippy::cast_possible_truncation)] // each cast is the store into a float
fn wedge(resolution: [i32; 2], fov: [f64; 2], gamma: f64) -> BuiltinMesh {
    let x_edges = bin_edges(resolution[0], fov[0], gamma);
    let y_edges = bin_edges(resolution[1], fov[1], gamma);
    let [r0, r1] = resolution.map(index);
    let mut vert = vec![0.0_f32; 3 * r0 * r1];
    let r = 1.0_f64;
    for i in 0..r0 {
        for j in 0..r1 {
            let a = 0.5 * (x_edges[i + 1] + x_edges[i]);
            let e = 0.5 * (y_edges[j + 1] + y_edges[j]);
            let at = 3 * (i * r1 + j);
            vert[at] = (r * e.cos() * a.sin()) as f32;
            vert[at + 1] = (r * e.sin()) as f32;
            vert[at + 2] = (-r * e.cos() * a.cos()) as f32;
        }
    }
    BuiltinMesh {
        vertex: vert,
        face: None,
        inertia: None,
    }
}

/// `mjCMesh::MakeRect` (`user_mesh.cc:2493-2532`): a point at each cell's
/// centre of a `resolution` grid over [-1, 1]² at z = -1, two faces between
/// each four neighbours; the mesh a shell.
#[allow(clippy::cast_possible_truncation)] // each cast is the store into a float
fn plate(resolution: [i32; 2]) -> BuiltinMesh {
    let [r0, r1] = resolution.map(index);
    let dx = 2.0 / f64::from(resolution[0]);
    let dy = 2.0 / f64::from(resolution[1]);
    let mut vert = vec![0.0_f32; 3 * r0 * r1];
    let mut face = vec![0_u32; 6 * (r0 - 1) * (r1 - 1)];
    for (i, x) in (0..resolution[0]).map(|i| (index(i), f64::from(i))) {
        for (j, y) in (0..resolution[1]).map(|j| (index(j), f64::from(j))) {
            let v = i * r1 + j;
            vert[3 * v] = (-1.0 + (x + 0.5) * dx) as f32;
            vert[3 * v + 1] = (-1.0 + (y + 0.5) * dy) as f32;
            vert[3 * v + 2] = -1.0;
            if i > 0 && j > 0 {
                let cell = (i - 1) * (r1 - 1) + j - 1;
                face[6 * cell..6 * cell + 6].copy_from_slice(
                    &[
                        (i - 1) * r1 + j - 1,
                        i * r1 + j - 1,
                        (i - 1) * r1 + j,
                        i * r1 + j,
                        (i - 1) * r1 + j,
                        i * r1 + j - 1,
                    ]
                    .map(id),
                );
            }
        }
    }
    BuiltinMesh {
        vertex: vert,
        face: Some(face),
        inertia: Some(MeshInertia::Shell),
    }
}

/// `mjCMesh::MakeCone` (`user_mesh.cc:2537-2560`): `nedge` points on the
/// unit circle at z = -1, then `nedge` on the circle of `radius` at z = 1,
/// or, for a radius of 0 or less, the apex (0, 0, 1); no faces.
#[allow(clippy::cast_possible_truncation)] // each cast is the store into a float
fn cone(nedge: i32, radius: f64) -> BuiltinMesh {
    let top = radius > 0.0;
    let n = index(nedge);
    let mut vert = vec![0.0_f32; 3 * (n + if top { n } else { 1 })];
    let angle = |i: i32| f64::from(2 * i) * PI / f64::from(nedge);
    for i in 0..nedge {
        let at = 3 * index(i);
        vert[at] = angle(i).cos() as f32;
        vert[at + 1] = angle(i).sin() as f32;
        vert[at + 2] = -1.0;
    }
    if top {
        for i in nedge..2 * nedge {
            let at = 3 * index(i);
            vert[at] = (radius * angle(i).cos()) as f32;
            vert[at + 1] = (radius * angle(i).sin()) as f32;
            vert[at + 2] = 1.0;
        }
    } else {
        vert[3 * n + 2] = 1.0;
    }
    BuiltinMesh {
        vertex: vert,
        face: None,
        inertia: None,
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::{
        MeshBuiltin, cone_sizes, grid_sizes, make_mesh, supersphere_sizes, supertorus_sizes,
    };
    use crate::types::MeshInertia;

    /// Whether this is MuJoCo's platform, macOS on arm64, where the golden's
    /// floats were printed and the libm is the one MuJoCo called.
    const MUJOCO_PLATFORM: bool = cfg!(all(target_os = "macos", target_arch = "aarch64"));

    /// FNV-1a 64 of `bytes`, as the golden's writer hashes them.
    fn fnv(bytes: impl IntoIterator<Item = u8>) -> String {
        let hash = bytes.into_iter().fold(0xcbf2_9ce4_8422_2325_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
        });
        format!("{hash:016x}")
    }

    /// The words of `line` after `key`.
    fn words<'a>(line: &'a str, key: &str) -> impl Iterator<Item = &'a str> {
        line.strip_prefix(key)
            .unwrap_or_else(|| panic!("{line:.40} is not {key:?}"))
            .split_whitespace()
    }

    /// MuJoCo 3.5.0's own making of each case in `tests/assets/mesh_builtin/
    /// mjs_makemesh.txt` (its header says how it was made) is
    /// [`make_mesh`]'s: its refusal, or every vertex coordinate, every face
    /// id, and the inertia mode. A coordinate is compared by its bits on
    /// MuJoCo's platform; elsewhere a sphere's, hemisphere's or plate's still
    /// is, as their arithmetic calls no libm, and a supersphere's,
    /// supertorus's, wedge's or cone's, whose `cos`, `sin` or `pow` give the
    /// platform's last bit, within 1e-6, which a NaN fails.
    #[test]
    fn every_case_makes_the_mesh_mujoco_makes() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/assets/mesh_builtin/mjs_makemesh.txt"
        );
        let golden = std::fs::read_to_string(path).unwrap();
        let mut cases = 0;
        for case in golden.split("\ncase ").skip(1) {
            let mut lines = case.lines();
            let head = lines.next().unwrap();
            let (keyword, params) = head.split_once(' ').unwrap();
            let builtin = MeshBuiltin::from_keyword(keyword).unwrap();
            let params: Vec<f64> = params
                .split_whitespace()
                .map(|w| w.parse().unwrap())
                .collect();
            let made = make_mesh(builtin, &params);
            cases += 1;
            let line = lines.next().unwrap();
            if let Some(refusal) = line.strip_prefix("err ") {
                assert_eq!(made, Err(refusal.to_owned()), "{head}");
                continue;
            }
            let made = made.unwrap_or_else(|e| panic!("{head}: {e}"));
            if line.starts_with("vfnv ") {
                let [hash, count]: [&str; 2] =
                    words(line, "vfnv ").collect::<Vec<_>>().try_into().unwrap();
                assert_eq!(made.vertex.len().to_string(), count, "{head}");
                assert_eq!(
                    fnv(made.vertex.iter().flat_map(|x| x.to_le_bytes())),
                    hash,
                    "{head}"
                );
            } else {
                let want: Vec<f32> = words(line, "v ")
                    .map(|w| f32::from_bits(u32::from_str_radix(w, 16).unwrap()))
                    .collect();
                assert_eq!(made.vertex.len(), want.len(), "{head}");
                let libm = matches!(
                    builtin,
                    MeshBuiltin::Supersphere
                        | MeshBuiltin::Supertorus
                        | MeshBuiltin::Wedge
                        | MeshBuiltin::Cone
                );
                for (k, (&got, &want)) in made.vertex.iter().zip(&want).enumerate() {
                    if MUJOCO_PLATFORM || !libm {
                        assert_eq!(got.to_bits(), want.to_bits(), "{head}: coordinate {k}");
                    } else {
                        assert!((got - want).abs() <= 1e-6, "{head}: coordinate {k}");
                    }
                }
            }
            // The golden's faces are the generator's, or the sentinel
            // `0 1 2` it was written beside where the generator writes none.
            let faces = made.face.clone().unwrap_or_else(|| vec![0, 1, 2]);
            let line = lines.next().unwrap();
            if line.starts_with("ffnv ") {
                let [hash, count]: [&str; 2] =
                    words(line, "ffnv ").collect::<Vec<_>>().try_into().unwrap();
                assert_eq!(faces.len().to_string(), count, "{head}");
                assert_eq!(
                    fnv(faces.iter().flat_map(|i| i.to_le_bytes())),
                    hash,
                    "{head}"
                );
            } else {
                let want: Vec<u32> = words(line, "f").map(|w| w.parse().unwrap()).collect();
                assert_eq!(faces, want, "{head}");
            }
            // Written `exact`, the mode is exact unless the generator sets one.
            let inertia = match made.inertia.unwrap_or(MeshInertia::Exact) {
                MeshInertia::Exact => "exact",
                MeshInertia::Shell => "shell",
                other => panic!("{head}: {other:?}"),
            };
            assert_eq!(
                lines.next(),
                Some(format!("inertia {inertia}").as_str()),
                "{head}"
            );
        }
        assert_eq!(cases, 87);
    }

    /// Where MuJoCo 3.5.0's C leaves a built-in mesh undefined, it is refused
    /// here (`D-MESH-BUILTIN-UNDEFINED`): a cone of fewer than 0 edges
    /// (measured: MuJoCo reports "vector", and at a radius of 0 stops with a
    /// segmentation fault) and a mesh whose sizes overflow MuJoCo's `int`
    /// (measured: "vector", "std::bad_alloc", or gigabytes allocated).
    #[test]
    fn inputs_mujoco_leaves_undefined_are_refused() {
        use MeshBuiltin::{Cone, Plate, Supersphere, Supertorus, Wedge};
        let cases: [(MeshBuiltin, &[f64], &str); 10] = [
            (Cone, &[-1.0, 0.5], "Cone edge count cannot be negative"),
            (Cone, &[-1.0, 0.0], "Cone edge count cannot be negative"),
            (Cone, &[-3.0, 0.5], "Cone edge count cannot be negative"),
            (Cone, &[1e9, 0.5], "Cone mesh is too large"),
            (Cone, &[2_147_483_647.0, 0.5], "Cone mesh is too large"),
            (
                Supersphere,
                &[20000.0, 1.0, 1.0],
                "Supersphere mesh is too large",
            ),
            (
                Supertorus,
                &[20000.0, 0.5, 1.0, 1.0],
                "Supertorus mesh is too large",
            ),
            (Plate, &[50000.0, 50000.0], "Plate mesh is too large"),
            (Plate, &[2_147_483_647.0, 1.0], "Plate mesh is too large"),
            (
                Wedge,
                &[50000.0, 50000.0, 90.0, 45.0, 0.0],
                "Wedge mesh is too large",
            ),
        ];
        for (builtin, params, refusal) in cases {
            assert_eq!(
                make_mesh(builtin, params),
                Err(refusal.to_owned()),
                "{builtin:?} {params:?}"
            );
        }
    }

    /// Each size check refuses exactly where MuJoCo's `int` arithmetic first
    /// overflows: the largest of each that holds its counts passes (checked
    /// without making the mesh).
    #[test]
    fn a_size_check_refuses_from_the_first_int_overflow() {
        assert_eq!(supersphere_sizes(18_919), Some(()));
        assert_eq!(supersphere_sizes(18_920), None);
        assert_eq!(supertorus_sizes(18_918), Some(()));
        assert_eq!(supertorus_sizes(18_919), None);
        assert_eq!(grid_sizes([1, 238_609_294]), Some(()));
        assert_eq!(grid_sizes([1, 238_609_295]), None);
        assert_eq!(grid_sizes([238_609_294, 1]), Some(()));
        assert_eq!(grid_sizes([238_609_295, 1]), None);
        assert_eq!(cone_sizes(357_913_941, 0.5), Some(()));
        assert_eq!(cone_sizes(357_913_942, 0.5), None);
        assert_eq!(cone_sizes(715_827_881, 0.0), Some(()));
        assert_eq!(cone_sizes(715_827_882, 0.0), None);
    }
}

//! Fluid force precomputation for MJCF geom processing.
//!
//! Computes the 12-element `geom_fluid` array used by sim-core's ellipsoidal
//! fluid interaction model. Added-mass coefficients are evaluated via 15-point
//! Gauss–Kronrod quadrature, matching MuJoCo's `GetAddedMassKappa` in
//! `user_objects.cc`.

use nalgebra::Vector3;
use sim_core::GeomType;

use crate::types::FluidShape;

/// 15-point Gauss-Kronrod integration constants from MuJoCo `user_objects.cc`.
/// Pre-transformed integration nodes: l_i = x_i³ / (1 - x_i)²
#[allow(clippy::unreadable_literal)]
const KRONROD_L: [f64; 15] = [
    7.865151709349917e-08,
    1.7347976913907274e-05,
    3.548008144506193e-04,
    2.846636252924549e-03,
    1.4094260903596077e-02,
    5.3063261727396636e-02,
    1.7041978741317773e-01,
    5.0e-01,
    1.4036301548686991e+00,
    3.9353484827022642e+00,
    1.1644841677041734e+01,
    3.953187807410903e+01,
    1.775711362220801e+02,
    1.4294772912937397e+03,
    5.4087416549217705e+04,
];

/// Gauss-Kronrod integration weights.
#[allow(clippy::unreadable_literal)]
const KRONROD_W: [f64; 15] = [
    0.01146766, 0.03154605, 0.05239501, 0.07032663, 0.08450236, 0.09517529, 0.10221647, 0.10474107,
    0.10221647, 0.09517529, 0.08450236, 0.07032663, 0.05239501, 0.03154605, 0.01146766,
];

/// Jacobian dl/dx evaluated at each node: x²(3-x)/(1-x)³
#[allow(clippy::unreadable_literal)]
const KRONROD_D: [f64; 15] = [
    5.538677720489877e-05,
    2.080868285293228e-03,
    1.6514126520723166e-02,
    7.261900344370877e-02,
    2.3985243401862602e-01,
    6.868318249020725e-01,
    1.8551129519182894e+00,
    5.0e+00,
    1.4060031152313941e+01,
    4.328941239611009e+01,
    1.5658546376397112e+02,
    7.479826085305024e+02,
    5.827404295002712e+03,
    1.167540197944512e+05,
    2.5482945327264845e+07,
];

/// Guard for degenerate virtual inertia (matches MuJoCo's mjEPS = 1e-14).
const MJ_EPS: f64 = 1e-14;

/// Compute kappa integral for one axis via 15-point Gauss-Kronrod quadrature.
/// `dx` is the "special" axis; `dy`, `dz` are the other two (interchangeable).
/// Matches MuJoCo's `GetAddedMassKappa` in `user_objects.cc`.
fn get_added_mass_kappa(dx: f64, dy: f64, dz: f64) -> f64 {
    let inv_dx2 = 1.0 / (dx * dx);
    let inv_dy2 = 1.0 / (dy * dy);
    let inv_dz2 = 1.0 / (dz * dz);
    let scale = (dx * dx * dx * dy * dz).powf(0.4); // (dx³·dy·dz)^(2/5)

    let mut kappa = 0.0;
    for i in 0..15 {
        let lambda = scale * KRONROD_L[i];
        let denom = (1.0 + lambda * inv_dx2)
            * ((1.0 + lambda * inv_dx2) * (1.0 + lambda * inv_dy2) * (1.0 + lambda * inv_dz2))
                .sqrt();
        kappa += scale * KRONROD_D[i] / denom * KRONROD_W[i];
    }
    kappa * inv_dx2
}

/// Compute semi-axes for a geom, matching MuJoCo's `mju_geomSemiAxes`.
pub fn geom_semi_axes(geom_type: GeomType, size: Vector3<f64>) -> [f64; 3] {
    match geom_type {
        GeomType::Sphere => [size.x, size.x, size.x],
        GeomType::Capsule => [size.x, size.x, size.y + size.x],
        GeomType::Cylinder => [size.x, size.x, size.y],
        _ => [size.x, size.y, size.z], // Ellipsoid, Box, Mesh
    }
}

/// Compute the 12-element `geom_fluid` array for a single geom.
/// For `FluidShape::None`: all zeros.
/// For `FluidShape::Ellipsoid`: element 0 = 1.0, elements 1-5 from fluidcoef
/// (or defaults), elements 6-11 from kappa quadrature.
pub fn compute_geom_fluid(
    fluidshape: FluidShape,
    fluidcoef: Option<[f64; 5]>,
    geom_type: GeomType,
    size: Vector3<f64>,
) -> [f64; 12] {
    if fluidshape == FluidShape::None {
        return [0.0; 12];
    }

    // Defaults: [C_blunt=0.5, C_slender=0.25, C_ang=1.5, C_K=1.0, C_M=1.0]
    let coef = fluidcoef.unwrap_or([0.5, 0.25, 1.5, 1.0, 1.0]);

    // Semi-axes (diameters in MuJoCo's convention = semi-axes here)
    let s = geom_semi_axes(geom_type, size);
    let dx = s[0];
    let dy = s[1];
    let dz = s[2];

    // Kappa integrals via cyclic permutation
    let kx = get_added_mass_kappa(dx, dy, dz);
    let ky = get_added_mass_kappa(dy, dz, dx);
    let kz = get_added_mass_kappa(dz, dx, dy);

    // Volume
    let vol = (4.0 / 3.0) * std::f64::consts::PI * dx * dy * dz;

    // Virtual mass: V * κ_i / max(EPS, 2 - κ_i)
    let vm = [
        vol * kx / (2.0 - kx).max(MJ_EPS),
        vol * ky / (2.0 - ky).max(MJ_EPS),
        vol * kz / (2.0 - kz).max(MJ_EPS),
    ];

    // Virtual inertia: V * I_fac / 5
    // I_x_fac = (dy²-dz²)² * |κ_z-κ_y| / max(EPS, |2(dy²-dz²)+(dy²+dz²)(κ_y-κ_z)|)
    #[allow(clippy::suspicious_operation_groupings)]
    let vi_fac = |da: f64, db: f64, ka: f64, kb: f64| -> f64 {
        let diff_sq = da * da - db * db;
        let sum_sq = da * da + db * db;
        let num = diff_sq * diff_sq * (kb - ka).abs();
        let den = (2.0 * diff_sq + sum_sq * (ka - kb)).abs();
        num / den.max(MJ_EPS)
    };
    let vi = [
        vol * vi_fac(dy, dz, ky, kz) / 5.0,
        vol * vi_fac(dz, dx, kz, kx) / 5.0,
        vol * vi_fac(dx, dy, kx, ky) / 5.0,
    ];

    [
        1.0,     // interaction_coef (master switch)
        coef[0], // C_blunt
        coef[1], // C_slender
        coef[2], // C_ang
        coef[3], // C_K (Kutta)
        coef[4], // C_M (Magnus)
        vm[0], vm[1], vm[2], vi[0], vi[1], vi[2],
    ]
}

"""Regenerate `scaled_moment_arms_opensim.json` — the real-OpenSim reference for
the A3 **differential oracle**: it grades our per-axis morph (`cf_msk_lib::realize`)
against OpenSim's own ScaleTool.

The keystone of A3 validation: a *dialed* body has no real subject, so there is no
moment-arm oracle for it — EXCEPT that our morph (per-segment, per-axis scaling of
bone offsets + muscle attachment points, joint coupling left symbolic) is exactly
what OpenSim's ScaleTool does. So for any scale vector we can scale gait2392 with
real OpenSim, compute moment arms, and grade our `realize`d model against it. This
turns "uniform scale = exact dilation (analytic)" into "anisotropic per-axis =
matches OpenSim's own scaling (empirical)".

Run (needs the `opensim` PyPI wheel):

    uv venv --python 3.12 /tmp/osim-venv
    uv pip install --python /tmp/osim-venv opensim
    /tmp/osim-venv/bin/python gen_scaled_moment_arms.py gait2392.osim scaled_moment_arms_opensim.json

Each config is per-body scale FACTORS as an OpenSim body-frame Vec3 [x, y, z],
where **y is axial (length)** and **x = z is transverse (girth)** — the gait2392
limb long axis is body-frame y. This ordering is `cf_msk_lib::SegmentScale.vector()`
= (transverse, axial, transverse), so the Rust cross-check reads these factors back
into a `BodyParams` directly. The knee is swept 0 -> -100 deg (the same ROM as
`gen_moment_arms.py`), all other coordinates at their defaults.
"""
import json
import math
import sys
import opensim as osim

osim.Logger.setLevelString("Error")  # silence missing-geometry warnings

OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]
ANGLES_DEG = [-(5.0 * i) for i in range(21)]  # 0, -5, ... -100

# --- The generator's own factor range (mirrors `cf_msk_lib::anthro`) ----------
# So the grid below can carry the ACTUAL per-axis factors `AnthroSource` emits at
# the percentile extremes — not just synthetic single-axis perturbations. This
# turns the A3-PR4 T1-coverage claim from "the generator's factors fall within a
# per-axis range we tested separately" into "the generator's own output at the
# extremes is directly OpenSim-graded." The table values and reference
# (50th-male) MUST match `cf-msk-lib/src/anthro.rs`; `a3_gate` asserts the Rust
# generator's factors land inside this grid's envelope, catching any drift.
TABLE = {  # (mean, sd) meters — ANSUR II stature; representative ANSUR girths
    "male":   {"stature": (1.756, 0.0686), "thigh": (0.620, 0.054), "calf": (0.377, 0.028)},
    "female": {"stature": (1.629, 0.0640), "thigh": (0.600, 0.056), "calf": (0.359, 0.029)},
}
REF_SEX, REF_PCT = "male", 0.5


def _probit(p):
    """Inverse standard-normal CDF (Acklam) — matches `anthro::probit`."""
    a = [-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
         1.38357751867269e+02, -3.066479806614716e+01, 2.506628277459239e+00]
    b = [-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
         6.680131188771972e+01, -1.328068155288572e+01]
    c = [-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
         -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00]
    d = [7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
         3.754408661907416e+00]
    plow, phigh = 0.02425, 1.0 - 0.02425
    if p < plow:
        q = math.sqrt(-2.0 * math.log(p))
        return (((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q+c[5]) / \
               ((((d[0]*q+d[1])*q+d[2])*q+d[3])*q+1.0)
    if p <= phigh:
        q = p - 0.5
        r = q*q
        return (((((a[0]*r+a[1])*r+a[2])*r+a[3])*r+a[4])*r+a[5])*q / \
               (((((b[0]*r+b[1])*r+b[2])*r+b[3])*r+b[4])*r+1.0)
    q = math.sqrt(-2.0 * math.log(1.0 - p))
    return -(((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q+c[5]) / \
            ((((d[0]*q+d[1])*q+d[2])*q+d[3])*q+1.0)


def _norm_at(meansd, p):
    mean, sd = meansd
    return mean + sd * _probit(p)


def gen_factors(sex, stature_pct, girth_pct):
    """The per-body Vec3 factors `AnthroSource::new(sex, stature_pct)
    .with_girth_percentile(girth_pct)` produces: axial = stature ratio (both
    segments), transverse = girth ratio."""
    t, ref = TABLE[sex], TABLE[REF_SEX]
    axial = _norm_at(t["stature"], stature_pct) / _norm_at(ref["stature"], REF_PCT)
    thigh = _norm_at(t["thigh"], girth_pct) / _norm_at(ref["thigh"], REF_PCT)
    calf = _norm_at(t["calf"], girth_pct) / _norm_at(ref["calf"], REF_PCT)
    return {"femur_r": (thigh, axial, thigh), "tibia_r": (calf, axial, calf)}


# Scale grid — single-axis perturbations (length=axial=y, girth=transverse=x,z on
# each segment) bracketing the generator's range on BOTH sides, a "realistic
# subject" mix, and the generator's ACTUAL coupled output at the sampled 1st/99th
# percentile extremes, both sexes. Bodies omitted stay at scale 1.
CONFIGS = {
    "uniform_1.137": {"pelvis": (1.137,) * 3, "femur_r": (1.137,) * 3,
                      "tibia_r": (1.137,) * 3, "talus_r": (1.137,) * 3},
    "femur_axial_1.2": {"femur_r": (1.0, 1.2, 1.0)},
    "femur_axial_0.9": {"femur_r": (1.0, 0.9, 1.0)},
    "femur_transverse_1.3": {"femur_r": (1.3, 1.0, 1.3)},
    "femur_transverse_0.85": {"femur_r": (0.85, 1.0, 0.85)},
    "tibia_axial_0.9": {"tibia_r": (1.0, 0.9, 1.0)},
    "tibia_transverse_1.2": {"tibia_r": (1.2, 1.0, 1.2)},
    "tibia_transverse_0.85": {"tibia_r": (0.85, 1.0, 0.85)},
    # A plausible taller/leaner subject: longer + slightly thicker thigh, longer +
    # slightly leaner shank.
    "realistic_mix": {"femur_r": (1.05, 1.10, 1.05), "tibia_r": (0.95, 1.08, 0.95)},
    # The generator's own coupled output at the SAMPLED percentile extremes
    # (`AnthroSource` advertises 0.01–0.99) — the strongest T1-coverage anchor:
    # OpenSim grades exactly what the generator emits. Both sexes at 1st/99th, so
    # the WHOLE coupled family across the advertised range lands inside the
    # envelope (female-1st is the lowest factor, male-99th the highest).
    "gen_male_01": gen_factors("male", 0.01, 0.01),
    "gen_male_99": gen_factors("male", 0.99, 0.99),
    "gen_female_01": gen_factors("female", 0.01, 0.01),
    "gen_female_99": gen_factors("female", 0.99, 0.99),
}


def moment_arms(factors):
    """Scale gait2392 by `factors` (per-body Vec3) and return knee moment arms."""
    model = osim.Model(OSIM)
    state = model.initSystem()
    if factors:
        scales = osim.ScaleSet()
        for body, (sx, sy, sz) in factors.items():
            sc = osim.Scale()
            sc.setSegmentName(body)
            sc.setScaleFactors(osim.Vec3(sx, sy, sz))
            sc.setApply(True)
            scales.cloneAndAppend(sc)
        if not model.scale(state, scales, True):  # preserveMassDist=True
            raise RuntimeError("model.scale returned False")
        state = model.initSystem()  # fresh state for the scaled model
    coord = model.getCoordinateSet().get("knee_angle_r")
    out = {}
    for m in MUSCLES:
        musc = osim.Muscle.safeDownCast(model.getForceSet().get(m))
        rows = []
        for adeg in ANGLES_DEG:
            arad = math.radians(adeg)
            coord.setValue(state, arad, True)  # enforce constraints
            model.realizePosition(state)
            rows.append({
                "angle_deg": adeg,
                "angle_rad": arad,
                "moment_arm_m": musc.computeMomentArm(state, coord),
            })
        out[m] = rows
    return out


configs = {}
for name, factors in CONFIGS.items():
    configs[name] = {
        "factors": {b: list(v) for b, v in factors.items()},
        "muscles": moment_arms(factors),
    }

with open(OUT, "w") as f:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "coordinate": "knee_angle_r",
        "convention": "moment_arm = -d(length)/d(coordinate), meters; "
                      "factors are body-frame Vec3 [x,y,z], y=axial(length), x=z=transverse(girth)",
        "configs": configs,
    }, f, indent=1)

print(f"OpenSim {osim.GetVersion()}  scaled-knee moment arms (mm) at 0 / 50 / 100 deg flexion:")
for name in CONFIGS:
    print(f"  [{name}]")
    for m in MUSCLES:
        r = configs[name]["muscles"][m]
        print(f"    {m:<11} {r[0]['moment_arm_m']*1000:7.2f} "
              f"{r[10]['moment_arm_m']*1000:7.2f} {r[20]['moment_arm_m']*1000:7.2f}")

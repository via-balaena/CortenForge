//! A 95A polyurethane tire cast onto a printed rim — the mold set, the bench
//! sheet, and the oracle that says both are sound.
//!
//! Two modes, one binary:
//!
//! - **No arguments** — build the stock wheel ([`WheelSpec::iter1`]) and assert
//!   the oracle below. This is the path `xtask run-validators` takes, so the
//!   export is executed on every CI run rather than type-checked.
//! - **With arguments** — a generator. Any geometry `WheelSpec` accepts,
//!   exported wherever you point it. The oracle's size-dependent half is
//!   reported rather than asserted, because a bigger wheel legitimately does
//!   not fit six tires in 2 lb of polyurethane.
//!
//! ## The oracle (stock geometry only)
//!
//! 1. The export writes exactly four STLs: two cup halves, the rim, the dowel.
//! 2. **No separate `funnel.stl`** — one would mean the apex-axial layout
//!    silently fell back to V-at-dome.
//! 3. The pour mass is the pinned 115.11 g.
//! 4. Six tires fit the 2 lb holdings with room for a retry.
//! 5. The sheet never tells the bencher to release the rim or pull it out.
//!
//! ⚠ (5) is not decoration. The rim is an INSERT: it stays inside the cured
//! tire. A silicone cast's two reflexes — mold-release the plug, then pull it
//! out — each destroy a wheel, and the release one fires before the pour.

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use cf_cast::wheel::{
    DimpleSpec, KeyingKind, NOMINAL_PU_95A_DENSITY_KG_M3, WheelSpec, nominal_tire_volume_m3,
    wheel_cast_spec, wheel_mold_ribbon,
};
use cf_cast::{DEFAULT_MASS_BUDGET_KG, STLS_SUBDIR};
use mesh_printability::PrinterConfig;

/// Cup-wall thickness the workshop prints at.
const WALL_M: f64 = 0.005;
/// Tires per trike set, times two sets. The number the budget is checked against.
const TIRES_WANTED: f64 = 6.0;
const MM_PER_M: f64 = 1000.0;
const G_PER_KG: f64 = 1000.0;
/// The stock wheel's reported pour mass. ★ ONE definition: the comparison and
/// the failure message both read it. Mutating the comparison alone once
/// produced "pour mass drifted: 115.11 g, expected 115.11 g" — a gate whose
/// message contradicted what it had actually tested.
const STOCK_POUR_G: f64 = 115.11;

const USAGE: &str = "\
example-cast-wheel — a PU tire cast onto a printed rim

USAGE:
    cargo run --release -p example-cast-wheel [-- OPTIONS]

OPTIONS (all lengths in mm; defaults are the stock wheel):
    --tire-od <mm>     tire outer diameter          [130]
    --rim-od <mm>      rim outer diameter           [105]
    --width <mm>       tread width                  [25]
    --bore <mm>        axle bore diameter           [8]
    --dimples <n>      keying dimples, 0 = none     [8]
    --dimple-r <mm>    dimple radius                [3]
    --cell-mm <mm>     marching-cubes cell          [2.0]
    --out <dir>        output directory             [<this crate>/out]
    --help

NOTES:
    --cell-mm is a SURFACE-QUALITY choice only. The reported pour mass is
    integrated on a cell floored at 2 mm, so 2.0, 1.5 and 1.0 all report the
    same grams. Finer cells cost time and faces: 2 mm ~1.4 s / 57 k faces per
    half, 1 mm ~4.4 s / 221 k. Print at 1 mm; iterate at 2 mm.

    Running with NO options asserts an oracle. Running with options does not,
    beyond what the export gate itself refuses.
";

struct Args {
    tire_od_mm: f64,
    rim_od_mm: f64,
    width_mm: f64,
    bore_mm: f64,
    dimples: u32,
    dimple_r_mm: f64,
    cell_mm: f64,
    out: PathBuf,
    /// Did any GEOMETRY flag get passed? The oracle's size-dependent half is
    /// asserted only on the stock wheel.
    ///
    /// ⚠ Set by the parser, NOT recovered by comparing the parsed values back
    /// against the defaults. That comparison was float equality on every
    /// dimension — which clippy is right to reject, and which would also have
    /// called `--tire-od 130` stock while the caller was plainly driving.
    /// `--cell-mm` and `--out` change no geometry and do not set it.
    stock: bool,
}

impl Default for Args {
    fn default() -> Self {
        // ★ Read OFF the spec rather than retyped, so the usage text and the
        // stock wheel cannot drift apart.
        let s = WheelSpec::iter1();
        let dimple = s.keying.dimple_spec();
        Self {
            tire_od_mm: s.tire_outer_radius_m * 2.0 * MM_PER_M,
            rim_od_mm: s.rim_outer_radius_m * 2.0 * MM_PER_M,
            width_mm: s.width_m * MM_PER_M,
            bore_mm: s.bore_radius_m * 2.0 * MM_PER_M,
            dimples: dimple.map_or(0, |d| d.count),
            dimple_r_mm: dimple.map_or(0.0, |d| d.radius_m * MM_PER_M),
            cell_mm: 2.0,
            stock: true,
            // ⚠ Anchored to the CRATE, not the cwd. `cargo run` runs from
            // the workspace ROOT, so a relative "out" drops a directory there
            // and dirties the tree — which it did, once.
            out: PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("out"),
        }
    }
}

impl Args {
    fn spec(&self) -> WheelSpec {
        let keying = if self.dimples == 0 {
            KeyingKind::None
        } else {
            KeyingKind::Dimples(DimpleSpec {
                count: self.dimples,
                radius_m: self.dimple_r_mm / MM_PER_M,
            })
        };
        WheelSpec {
            tire_outer_radius_m: self.tire_od_mm / 2.0 / MM_PER_M,
            rim_outer_radius_m: self.rim_od_mm / 2.0 / MM_PER_M,
            bore_radius_m: self.bore_mm / 2.0 / MM_PER_M,
            width_m: self.width_mm / MM_PER_M,
            keying,
            ..WheelSpec::iter1()
        }
    }
}

/// `None` means `--help` was printed and there is nothing to do.
fn parse() -> Result<Option<Args>> {
    let mut args = Args::default();
    let mut input = std::env::args().skip(1);
    while let Some(flag) = input.next() {
        // ⚠ Every arm takes a value except `--help`; a missing value must be
        // an error, not a silent default.
        let mut value = || -> Result<String> {
            input
                .next()
                .with_context(|| format!("{flag} needs a value"))
        };
        let mm = |v: String| -> Result<f64> {
            let n: f64 = v
                .parse()
                .with_context(|| format!("{v:?} is not a number"))?;
            if !n.is_finite() || n <= 0.0 {
                bail!("{v:?} must be a positive, finite length in mm");
            }
            Ok(n)
        };
        match flag.as_str() {
            "--help" | "-h" => {
                print!("{USAGE}");
                return Ok(None);
            }
            "--tire-od" => {
                args.tire_od_mm = mm(value()?)?;
                args.stock = false;
            }
            "--rim-od" => {
                args.rim_od_mm = mm(value()?)?;
                args.stock = false;
            }
            "--width" => {
                args.width_mm = mm(value()?)?;
                args.stock = false;
            }
            "--bore" => {
                args.bore_mm = mm(value()?)?;
                args.stock = false;
            }
            "--dimple-r" => {
                args.dimple_r_mm = mm(value()?)?;
                args.stock = false;
            }
            "--cell-mm" => args.cell_mm = mm(value()?)?,
            "--dimples" => {
                let v = value()?;
                args.dimples = v.parse().with_context(|| format!("{v:?} is not a count"))?;
                args.stock = false;
            }
            "--out" => args.out = PathBuf::from(value()?),
            other => bail!("unknown option {other:?}\n\n{USAGE}"),
        }
    }
    Ok(Some(args))
}

fn main() -> Result<()> {
    // `WheelSpec`'s orderings are enforced by assertions inside cf-cast — one
    // definition, which is the point. Their MESSAGES are good ("tire outer
    // radius (0.065) must exceed rim outer radius (0.07)"); the backtrace
    // boilerplate around them is not, and this is a tool people will hold
    // wrong on purpose while exploring geometry.
    std::panic::set_hook(Box::new(|info| {
        let msg = info
            .payload()
            .downcast_ref::<String>()
            .map(String::as_str)
            .or_else(|| info.payload().downcast_ref::<&str>().copied())
            .unwrap_or("invalid wheel geometry");
        eprintln!("error: {msg}");
    }));

    let Some(args) = parse()? else { return Ok(()) };
    let spec = args.spec();
    // ⚠ Printed BEFORE the ribbon build, which is where `WheelSpec`'s
    // invariant assertions fire. Their messages name RADII IN METRES; this
    // line names the DIAMETERS IN MM the caller actually typed, so a rejected
    // wheel shows both halves of the conversion.
    println!(
        "wheel: tire OD {:.1} mm, rim OD {:.1} mm, tread {:.1} mm, bore {:.1} mm, {} dimple(s)",
        args.tire_od_mm, args.rim_od_mm, args.width_mm, args.bore_mm, args.dimples
    );
    let ribbon = wheel_mold_ribbon(&spec).context("build the wheel's mold ribbon")?;
    let cast = wheel_cast_spec(
        &spec,
        WALL_M,
        args.cell_mm / MM_PER_M,
        PrinterConfig::fdm_default(),
    );

    let report = cast
        .export_molds_v2(&ribbon, &args.out)
        .context("export the mold set (F4 printability gate runs here)")?;
    let sheet_path = args.out.join("procedure.md");
    cast.write_procedure_v2(&ribbon, &sheet_path)
        .context("write the bench sheet")?;
    let sheet = std::fs::read_to_string(&sheet_path).context("read back the bench sheet")?;

    let mut stls: Vec<String> = std::fs::read_dir(args.out.join(STLS_SUBDIR))
        .context("list the written STLs")?
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    stls.sort();

    let pour_g = report.layers[0].pour_volume.pour_mass_kg * G_PER_KG;
    let analytic_g = nominal_tire_volume_m3(&spec) * NOMINAL_PU_95A_DENSITY_KG_M3 * G_PER_KG;
    let budget_g = DEFAULT_MASS_BUDGET_KG * G_PER_KG;

    println!("cell:  {:.2} mm  →  {}", args.cell_mm, args.out.display());
    println!("files: {}", stls.join(", "));
    println!("pour:  {pour_g:.2} g reported, {analytic_g:.2} g closed-form");
    // ⚠ Report the SIGNED difference and stop there. The first draft of this
    // line said "UNDER-READS by {x} %" with the stock wheel's explanation
    // baked in — and printed "UNDER-READS by -0.2 %" on the first custom rim
    // anyone tried. The bias is floored at a 2 mm cell, so its SIGN and size
    // depend on the section; only the stock figure has been measured.
    println!(
        "       integrator vs closed form: {:+.2} % (cell floored at 2 mm; the sheet\n\
         \x20        prints the reported figure, so that is the one to weigh against)",
        (pour_g - analytic_g) / analytic_g * 100.0
    );
    println!(
        "budget: {:.0} tires fit the {budget_g:.0} g holdings ({:.0} g for {TIRES_WANTED:.0})",
        (budget_g / pour_g).floor(),
        pour_g * TIRES_WANTED
    );

    if !args.stock {
        println!("\ncustom geometry — the oracle below is NOT asserted, only the export gate ran.");
        return Ok(());
    }

    // ── the oracle ────────────────────────────────────────────────────────
    let expected = [
        "dowel.stl",
        "mold_layer_0_piece_0.stl",
        "mold_layer_0_piece_1.stl",
        "plug_layer_0.stl",
    ];
    if stls != expected {
        bail!("STL roster changed: expected {expected:?}, wrote {stls:?}");
    }
    if report.funnel.is_some() {
        bail!("a separate funnel.stl means apex-axial fell back to V-at-dome");
    }
    if (pour_g - STOCK_POUR_G).abs() > 0.01 {
        bail!("pour mass drifted: {pour_g:.2} g, expected {STOCK_POUR_G:.2} g");
    }
    if pour_g * TIRES_WANTED >= budget_g {
        bail!(
            "{TIRES_WANTED} tires ({:.0} g) no longer fit {budget_g:.0} g",
            pour_g * TIRES_WANTED
        );
    }
    // ⚠ Assert the sentence PREFIX, never the noun — `contains` matches the
    // WHOLE sheet, and "mold release" must survive for the CUP HALVES.
    for gone in [
        "Pull the plug",
        "off the plug",
        "Apply mold release to all printed",
    ] {
        if sheet.contains(gone) {
            bail!("the sheet still treats the rim as tooling: {gone:?}");
        }
    }
    for needed in [
        "2. Apply mold release to the two cup halves.",
        "`plug_layer_0.stl` gets NO mold release",
        "Leave the plug IN",
    ] {
        if !sheet.contains(needed) {
            bail!("the sheet lost an overmold instruction: {needed:?}");
        }
    }

    println!("\noracle: PASS — roster, integral funnel, pour mass, budget, and the sheet's");
    println!("        overmold instructions all hold on the stock wheel.");
    Ok(())
}

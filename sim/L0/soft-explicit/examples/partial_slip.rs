//! One run of K6, the cylinder pressed into the block and pushed sideways
//! (plan §16b), printed as one line: K6's errors per row and phase, the
//! validity gates, and the run's cost.
//!
//! `cargo run --release -p sim-soft-explicit --example partial_slip --
//! <a/h> <f32|f64> <R/a> <block/a> <viscous> <slowdown> [table]`
//! (defaults: 50 f64 100 10 0 1, which is `PartialSlipRun::plan`). `a/h` is
//! the fine region's elements per contact half-width (plan: 50; CI: 12);
//! `R/a` the cylinder's radius (the finite-strain companion: 200); `block/a`
//! the block's depth, its width twice that (the finite-domain companion: 15);
//! `viscous` the Kelvin–Voigt `η/μ` in seconds; `slowdown` divides every
//! speed and multiplies every time of the loading (the rate ladder: 2, 4, …).
//! With `table`, every sample is printed too. Set `RAYON_NUM_THREADS` so the
//! times are comparable.
//!
//! `… --example partial_slip -- compare <table> <table>` compares two runs'
//! printed tables: each judged sample's stick zone in the second against the
//! first's, interpolated to the same load fraction, printed as the largest
//! difference per phase. It is the rate ladder's measure and the companions'
//! (plan §16b).

#![allow(missing_docs, clippy::unwrap_used, clippy::cast_precision_loss)]

use std::time::Instant;

use sim_soft_explicit::cpu;
use sim_soft_explicit::fixtures::partial_slip::{Leg, PartialSlipResult, PartialSlipRun};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|a| a == "compare") {
        compare(&args[1], &args[2]);
        return;
    }
    let arg = |i: usize, default: &str| args.get(i).cloned().unwrap_or_else(|| default.to_owned());
    let divisions: f64 = arg(0, "50").parse().unwrap();
    let wide = arg(1, "f64") == "f64";
    let radius: f64 = arg(2, "100").parse().unwrap();
    let size: f64 = arg(3, "10").parse().unwrap();
    let viscous: f64 = arg(4, "0").parse().unwrap();
    let slowdown: f64 = arg(5, "1").parse().unwrap();
    let table = arg(6, "") == "table";

    let mut run = PartialSlipRun::plan(divisions).slowed(slowdown);
    let a = run.block.contact_half_width;
    run.cylinder.radius = radius * a;
    run.block.half_width = size * a;
    run.block.depth = size * a;
    run.viscous_time = viscous;
    let started = Instant::now();
    let result = if wide {
        run.run(|m, o| cpu::f64::CpuExecutor::new(m, o).unwrap())
    } else {
        run.run(|m, o| cpu::f32::CpuExecutor::new(m, o).unwrap())
    };
    let seconds = started.elapsed().as_secs_f64();
    match result {
        Ok(result) => report(&run, &result, seconds, table),
        Err(error) => println!("a/h={divisions} failed after {seconds:.1} s: {error}"),
    }
}

fn report(run: &PartialSlipRun, result: &PartialSlipResult, seconds: f64, table: bool) {
    let a = run.block.contact_half_width;
    if table {
        for reading in &result.readings {
            let fraction = result.fraction(reading).unwrap_or(f64::NAN);
            let rows = reading.rows.map(|r| {
                (
                    r.contact.map_or(f64::NAN, |z| z.half_width() / a),
                    r.stick_over_contact().unwrap_or(f64::NAN),
                )
            });
            println!(
                "{:?} t={:.5} P={:.5e} Q={:.5e} fraction={fraction:.4} a/A=({:.4},{:.4}) c/a=({:.4},{:.4})",
                reading.leg,
                reading.time,
                reading.normal_force,
                reading.tangential_force,
                rows[0].0,
                rows[1].0,
                rows[0].1,
                rows[1].1,
            );
        }
    }
    let errors = result.errors(0.2, 0.8);
    let show = |e: [Option<f64>; 2]| {
        e.map(|v| v.map_or_else(|| "none".to_owned(), |v| format!("{v:.4}")))
            .join("/")
    };
    let optional = |v: Option<f64>| v.map_or_else(|| "none".to_owned(), |v| format!("{v:.2e}"));
    let contact_ke = result
        .samples
        .iter()
        .map(|s| s.monitors.contact_kinetic_energy)
        .fold(0.0, f64::max);
    let last = |leg: Leg| result.readings.iter().rfind(|r| r.leg == leg);
    let pressed = last(Leg::Press).map_or(f64::NAN, |s| {
        let [f, b] = s
            .rows
            .map(|r| r.contact.map_or(f64::NAN, |z| z.half_width()));
        0.5 * (f + b) / a
    });
    let peak = result.peak_tangential_force
        / (run.friction * last(Leg::Push).map_or(f64::NAN, |r| r.normal_force));
    // What the press leaves along x, before any push (by symmetry it would be
    // none; the Kuhn split is not symmetric in x).
    let left = last(Leg::Press).map_or(f64::NAN, |r| {
        r.tangential_force / (run.friction * r.normal_force)
    });
    println!(
        "a/h={} block={}a R={}a eta/mu={} K6 loading={} unloading={} worst={} | pressed a={pressed:.4}a \
         press depth={:.4}a pressed Q/(f P)={left:.4} peak Q/(f P)={peak:.4} KE/IE={} balance={} inverted={} penetration={:.2e}a \
         contact_KE={contact_ke:.2e} | steps={} dt={:.3e} readings={} time={seconds:.1}s",
        a / run.block.fine,
        run.block.depth / a,
        run.cylinder.radius / a,
        run.viscous_time,
        show(errors.loading),
        show(errors.unloading),
        errors
            .worst()
            .map_or_else(|| "none".to_owned(), |w| format!("{w:.4}")),
        result.press_depth / a,
        optional(result.kinetic_over_internal),
        optional(result.energy_balance),
        result.inverted,
        result.max_penetration / a,
        result.steps,
        result.dt,
        result.readings.len(),
    );
}

/// One judged reading from a printed table: its leg, load fraction, and the
/// rows' stick half-width over the contact's.
fn parse(line: &str) -> Option<(Leg, f64, [f64; 2])> {
    let leg = match line.split_whitespace().next()? {
        "Push" => Leg::Push,
        "Return" => Leg::Return,
        _ => return None,
    };
    let field = |name: &str| line.split(name).nth(1)?.split_whitespace().next();
    let fraction: f64 = field("fraction=")?.parse().ok()?;
    let rows = field("c/a=(")?.trim_end_matches(')');
    let (front, back) = rows.split_once(',')?;
    Some((leg, fraction, [front.parse().ok()?, back.parse().ok()?]))
}

/// The largest difference between the second table's stick zones and the
/// first's, at the second's judged load fractions, the first's interpolated
/// linearly in the fraction.
fn compare(first: &str, second: &str) {
    let read = |path: &str| -> Vec<(Leg, f64, [f64; 2])> {
        let text = std::fs::read_to_string(path).unwrap();
        text.lines()
            .filter_map(parse)
            .filter(|r| (0.2..=0.8).contains(&r.1))
            .collect()
    };
    let (theirs, ours) = (read(first), read(second));
    let mut worst = [0.0_f64; 2];
    for (slot, leg) in [Leg::Push, Leg::Return].into_iter().enumerate() {
        let mut other: Vec<(f64, [f64; 2])> = theirs
            .iter()
            .filter(|r| r.0 == leg)
            .map(|r| (r.1, r.2))
            .collect();
        other.sort_by(|a, b| a.0.total_cmp(&b.0));
        for &(_, f, mine) in ours.iter().filter(|r| r.0 == leg) {
            let Some(i) = other.iter().position(|o| o.0 >= f).filter(|&i| i > 0) else {
                continue;
            };
            let ((f0, c0), (f1, c1)) = (other[i - 1], other[i]);
            let t = if f1 > f0 { (f - f0) / (f1 - f0) } else { 0.0 };
            for row in 0..2 {
                let interpolated = c0[row] + t * (c1[row] - c0[row]);
                worst[slot] = worst[slot].max((mine[row] - interpolated).abs());
            }
        }
    }
    println!(
        "{second} against {first}: largest difference in c/a, loading {:.4}, unloading {:.4}",
        worst[0], worst[1]
    );
}

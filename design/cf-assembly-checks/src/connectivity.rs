//! Whether each part is one body, or several pretending to be.
//!
//! A [`Part`](cf_design::Part) is a single rigid body to the solver and a
//! single object to a fabricator. Nothing in `cf-design` requires its solid to
//! be *connected*, and `Solid::union` will happily hold two shapes that never
//! touch — so a part can be geometrically absurd while every number about it
//! is correct.
//!
//! ## ⛔⛔ Why no other check here can see this
//!
//! Mass integrates fine over two lumps. A convergence check against a finer
//! grid agrees with itself. Interpenetration is about pairs, not insides.
//! Section modulus samples one plane. The member screen reads a load. Every one
//! of those questions has a sensible answer for an object in two pieces.
//!
//! Measured on the trike the day this was written: both front wheels were a
//! 6 mm rim hoop and a 30 mm hub plug with **180 mm of nothing between them**,
//! and had been through five review passes — including a fourteen-agent one
//! with dedicated numerics and robustness lenses. A gate asserting the barrel
//! was wider than the hub passed, because it compared a proportion and never
//! asked whether the object was one thing.
//!
//! ## ⚠ What a positive means, and what it does not
//!
//! This samples and flood-fills, so it inherits the sampler's limits. It uses
//! **26-neighbour** connectivity — faces, edges and corners — deliberately, to
//! be forgiving: a thin wall crossing the grid diagonally can fragment under
//! 6-neighbour filling and read as disconnected when it is not. The bias is
//! toward silence, so a report here is worth believing and silence is worth
//! less than it looks.

use std::collections::{HashMap, VecDeque};

use cf_design::mechanism::Mechanism;
use nalgebra::Point3;

/// A part whose sampled interior falls into more than one component.
#[derive(Debug, Clone, PartialEq)]
pub struct Split {
    /// The part.
    pub part: String,
    /// How many separate bodies its solid describes.
    pub components: usize,
    /// Share of the sampled interior lying in the largest of them, `0.0..=1.0`.
    ///
    /// ★ Near `1.0` means one body plus specks — often a sampling artefact at
    /// a corner. Near `0.5` means the part is genuinely two objects.
    pub largest_share: f64,
}

/// What the connectivity pass found, and what it could not read.
#[derive(Debug, Clone, PartialEq)]
pub struct Bodies {
    /// Every part that is more than one body.
    pub split: Vec<Split>,
    /// Parts whose interior did not sample at this cell, and so were not
    /// judged either way.
    ///
    /// ⛔ **Unread, not sound.**
    pub unreadable: Vec<String>,
    /// How many parts were successfully examined.
    ///
    /// ⚠ Carried so the caller has the DENOMINATOR. A list of findings with no
    /// population behind it reads as "all clear" whatever it omitted.
    pub examined: usize,
}

/// The sampling step to use for each part, by name.
///
/// ⚠⚠ **Per part, not one figure for the assembly**, because no single cell
/// works: a 2300 mm rail needs a coarse one to fit in memory and a 2 mm tube
/// wall needs a fine one to survive the fill at all. cf-trike already picks a
/// cell per part to integrate its mass against, and that is the number to
/// reuse — the caller knows its own geometry, and this crate does not.
pub type Cells = HashMap<String, f64>;

/// The largest grid this will allocate, in cells.
///
/// One byte a cell, so this is the memory ceiling in bytes as well.
const MAX_CELLS: usize = 64_000_000;

/// Cells across the smallest dimension of a part before its sampling is
/// considered too coarse to judge.
const MIN_CELLS_ACROSS: f64 = 4.0;

/// Find every part that is secretly more than one body.
///
/// Each part is sampled at its own cell from `cells`; one with no entry, or
/// whose cell cannot resolve it, is reported unreadable rather than judged.
#[must_use]
pub fn disconnected_parts(mechanism: &Mechanism, cells: &Cells) -> Bodies {
    let mut split = Vec::new();
    let mut unreadable = Vec::new();
    let mut examined = 0;

    for part in mechanism.parts() {
        let Some(&cell_mm) = cells.get(part.name()) else {
            unreadable.push(part.name().to_owned());
            continue;
        };
        match components_of(part.solid(), cell_mm) {
            None => unreadable.push(part.name().to_owned()),
            Some(sizes) => {
                examined += 1;
                if sizes.len() > 1 {
                    let total: usize = sizes.iter().sum();
                    let largest = sizes.iter().copied().max().unwrap_or(0);
                    #[allow(clippy::cast_precision_loss)]
                    let share = largest as f64 / total as f64;
                    split.push(Split {
                        part: part.name().to_owned(),
                        components: sizes.len(),
                        largest_share: share,
                    });
                }
            }
        }
    }
    split.sort_by(|a, b| a.largest_share.total_cmp(&b.largest_share));
    unreadable.sort();
    Bodies {
        split,
        unreadable,
        examined,
    }
}

/// Sizes of each connected component of a solid's sampled interior, or `None`
/// when the sampling is too coarse to say anything.
fn components_of(solid: &cf_design::Solid, cell_mm: f64) -> Option<Vec<usize>> {
    if !cell_mm.is_finite() || cell_mm <= 0.0 {
        return None;
    }
    let bounds = solid.bounds()?;
    let span = bounds.max - bounds.min;
    if span.min() / cell_mm < MIN_CELLS_ACROSS {
        return None;
    }

    // ⚠⚠ **Coarsen to fit rather than refusing.** The caller's cell is the one
    // its mass was integrated at, which is far finer than detecting a gap
    // needs — and at that cell the biggest parts overflow any sane grid. On
    // the trike, refusing outright left five parts unread, among them two
    // wishbones and the rear tyre, which is not a silence worth keeping.
    //
    // ⛔ It coarsens only as far as [`MIN_CELLS_ACROSS`] allows. Past that the
    // fill cannot tell a gap from a sampling artefact, and the part goes back
    // to being unread rather than guessed at.
    let mut cell = cell_mm;
    let (nx, ny, nz) = loop {
        let n = |extent: f64| -> usize {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let c = (extent / cell).ceil() as usize;
            c.max(1)
        };
        let (nx, ny, nz) = (n(span.x), n(span.y), n(span.z));
        if nx.saturating_mul(ny).saturating_mul(nz) <= MAX_CELLS {
            break (nx, ny, nz);
        }
        cell *= 2.0;
        if span.min() / cell < MIN_CELLS_ACROSS {
            return None;
        }
    };
    let cell_mm = cell;

    let at = |ix: usize, iy: usize, iz: usize| -> Point3<f64> {
        #[allow(clippy::cast_precision_loss)]
        let f = |i: usize| (i as f64 + 0.5) * cell_mm;
        Point3::new(
            bounds.min.x + f(ix),
            bounds.min.y + f(iy),
            bounds.min.z + f(iz),
        )
    };
    let idx = |ix: usize, iy: usize, iz: usize| (iz * ny + iy) * nx + ix;

    let mut inside = vec![false; nx * ny * nz];
    let mut any = false;
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                if solid.evaluate(&at(ix, iy, iz)) < 0.0 {
                    inside[idx(ix, iy, iz)] = true;
                    any = true;
                }
            }
        }
    }
    if !any {
        return None;
    }

    // 26-neighbour flood fill: faces, edges AND corners. See the module note —
    // the looser rule is deliberate, so a thin diagonal wall does not fragment
    // into false findings.
    let mut seen = vec![false; inside.len()];
    let mut sizes = Vec::new();
    for start in 0..inside.len() {
        if !inside[start] || seen[start] {
            continue;
        }
        let mut size = 0usize;
        let mut queue = VecDeque::from([start]);
        seen[start] = true;
        while let Some(cur) = queue.pop_front() {
            size += 1;
            let ix = cur % nx;
            let iy = (cur / nx) % ny;
            let iz = cur / (nx * ny);
            for dz in -1isize..=1 {
                for dy in -1isize..=1 {
                    for dx in -1isize..=1 {
                        if dx == 0 && dy == 0 && dz == 0 {
                            continue;
                        }
                        let (Some(jx), Some(jy), Some(jz)) = (
                            ix.checked_add_signed(dx).filter(|v| *v < nx),
                            iy.checked_add_signed(dy).filter(|v| *v < ny),
                            iz.checked_add_signed(dz).filter(|v| *v < nz),
                        ) else {
                            continue;
                        };
                        let j = idx(jx, jy, jz);
                        if inside[j] && !seen[j] {
                            seen[j] = true;
                            queue.push_back(j);
                        }
                    }
                }
            }
        }
        sizes.push(size);
    }
    Some(sizes)
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use cf_design::{JointDef, JointKind, Material, Mechanism, Part, Solid};
    use nalgebra::Vector3;

    use super::*;

    fn steel() -> Material {
        Material::new("steel", 7850.0)
    }

    fn cells(at: f64) -> Cells {
        Cells::from([("a".to_owned(), at), ("b".to_owned(), at)])
    }

    fn two_part(a: Solid, b: Solid) -> Mechanism {
        Mechanism::builder("t")
            .part(Part::new("a", a, steel()))
            .part(Part::new("b", b, steel()))
            .joint(JointDef::new(
                "j",
                "a",
                "b",
                JointKind::Fixed,
                Point3::origin(),
                Vector3::y(),
            ))
            .build()
    }

    /// ⛔⛔ The trike's own front wheel: a rim hoop and a hub plug with
    /// 180 mm of nothing between them, declared as one part.
    #[test]
    fn a_hoop_and_a_plug_that_never_touch_are_two_bodies() {
        let rim_o = 17.0 * 25.4 / 2.0;
        let hoop = Solid::cylinder(rim_o, 100.0).subtract(Solid::cylinder(rim_o - 6.0, 120.0));
        let plug = Solid::cylinder(30.0, 25.0);
        let m = two_part(hoop.union(plug), Solid::sphere(10.0));

        let found = disconnected_parts(&m, &cells(4.0));
        let wheel = found
            .split
            .iter()
            .find(|s| s.part == "a")
            .expect("a hoop and a plug 180 mm apart are two bodies");
        assert_eq!(wheel.components, 2, "{wheel:?}");
        assert!(
            wheel.largest_share < 1.0,
            "both components should hold real volume: {wheel:?}"
        );
        assert!(found.examined >= 1, "the denominator must be carried");
    }

    /// The same wheel with material spanning the gap is ONE body — so the
    /// check is not merely reporting every part that has a hole in it.
    #[test]
    fn a_hoop_and_a_plug_joined_by_a_web_are_one_body() {
        let rim_o = 17.0 * 25.4 / 2.0;
        let hoop = Solid::cylinder(rim_o, 100.0).subtract(Solid::cylinder(rim_o - 6.0, 120.0));
        let plug = Solid::cylinder(30.0, 25.0);
        let web = Solid::cylinder(rim_o, 8.0);
        let m = two_part(hoop.union(plug).union(web), Solid::sphere(10.0));

        assert!(
            !disconnected_parts(&m, &cells(4.0))
                .split
                .iter()
                .any(|s| s.part == "a"),
            "a wheel with a face is one body"
        );
    }

    /// ⛔ A part too thin to sample at all is UNREAD, not sound.
    #[test]
    fn a_part_too_thin_to_sample_is_named_not_passed() {
        let m = two_part(
            Solid::cuboid(Vector3::new(100.0, 100.0, 0.4)),
            Solid::sphere(10.0),
        );
        let found = disconnected_parts(&m, &cells(4.0));
        assert!(found.unreadable.contains(&"a".to_owned()), "got {found:?}");
        assert!(
            !found.split.iter().any(|s| s.part == "a"),
            "an unread part must not also be reported as split"
        );
    }

    /// ⛔⛔ And a part that DOES sample but is too coarse to flood-fill
    /// honestly is also unread — which is a different door, and the one
    /// [`MIN_CELLS_ACROSS`] exists for.
    ///
    /// ⚠ Written after a mutation showed the test above passing with that
    /// guard deleted: a 0.4 mm plate samples to nothing and is caught by the
    /// empty-interior path instead, so the guard had no witness at all. Three
    /// cells across a wall is not enough to tell a gap from a sampling
    /// artefact, and reporting "one body" from it would be a guess.
    #[test]
    fn a_part_that_samples_but_is_too_coarse_to_judge_is_also_unread() {
        // 20 mm thick at a 4 mm cell is five cells; 10 mm is two and a half.
        let thick = two_part(
            Solid::cuboid(Vector3::new(100.0, 100.0, 20.0)),
            Solid::sphere(10.0),
        );
        let coarse = two_part(
            Solid::cuboid(Vector3::new(100.0, 100.0, 5.0)),
            Solid::sphere(10.0),
        );
        assert!(
            !disconnected_parts(&thick, &cells(4.0))
                .unreadable
                .contains(&"a".to_owned()),
            "five cells across is enough to judge"
        );
        let shallow = disconnected_parts(&coarse, &cells(4.0));
        assert!(
            shallow.unreadable.contains(&"a".to_owned()),
            "two cells across is not enough to tell a gap from an artefact: {shallow:?}"
        );
    }

    /// ★★★ Why the fill is 26-neighbour and not 6, measured rather than
    /// asserted.
    ///
    /// A thin plate crossing the grid diagonally is ONE body, and under
    /// face-only connectivity its cells touch at edges and corners rather than
    /// faces — so it shatters. Measured on a 60 x 60 plate at 45 degrees:
    ///
    /// ```text
    ///  plate  cell   26-neighbour   6-neighbour
    ///   3 mm   2 mm       1              1
    ///   2 mm  1.5 mm      1             57
    ///   4 mm   3 mm       1             29
    ///   1 mm   1 mm       1             85
    /// ```
    ///
    /// ⚠ Eighty-five findings from one sound object. A check that noisy gets
    /// switched off, and a check that is switched off is worth less than none
    /// — so the looser rule is the whole reason this is usable.
    ///
    /// ⚠⚠ This test exists because a mutation to 6-neighbour left the
    /// wheel-with-a-web case passing: the justification was in a doc comment
    /// with nothing behind it.
    #[test]
    fn a_thin_diagonal_plate_is_one_body_not_eighty_five() {
        use nalgebra::UnitQuaternion;
        let plate = Solid::cuboid(Vector3::new(60.0, 60.0, 0.5)).rotate(
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), f64::to_radians(45.0)),
        );
        let m = two_part(plate, Solid::sphere(10.0));
        let found = disconnected_parts(&m, &cells(1.0));
        assert!(
            !found.split.iter().any(|s| s.part == "a"),
            "a sound plate fragmented into {:?} — the fill is too strict",
            found.split
        );
    }

    /// ⛔⛔ A part too large to grid at the caller's cell is COARSENED to fit,
    /// not refused.
    ///
    /// ⚠ Written after the trike left five parts unread — two wishbones and
    /// the rear tyre among them — purely because their mass-integration cell
    /// was far finer than detecting a 180 mm gap needs. Refusing to look is
    /// the same failure as looking and saying nothing.
    #[test]
    fn a_part_too_big_to_grid_is_coarsened_rather_than_refused() {
        // A metre of tube at a tenth of a millimetre is billions of cells.
        let tube = Solid::cylinder(20.0, 500.0).subtract(Solid::cylinder(18.0, 520.0));
        let m = two_part(tube, Solid::sphere(10.0));
        let found = disconnected_parts(&m, &cells(0.1));
        assert!(
            !found.unreadable.contains(&"a".to_owned()),
            "it should have coarsened to fit: {found:?}"
        );
        assert!(
            !found.split.iter().any(|s| s.part == "a"),
            "a plain tube is one body: {found:?}"
        );
        assert_eq!(found.examined, 2);
    }

    /// ⛔ A part the caller gave no cell for is UNREAD, not sound — the same
    /// rule as everywhere else in this crate: an absent input names the part
    /// rather than shrinking the population quietly.
    #[test]
    fn a_part_with_no_cell_is_named_not_passed_over() {
        let m = two_part(Solid::sphere(40.0), Solid::sphere(10.0));
        let partial = Cells::from([("a".to_owned(), 3.0)]);
        let found = disconnected_parts(&m, &partial);
        assert!(found.unreadable.contains(&"b".to_owned()), "got {found:?}");
        assert_eq!(found.examined, 1, "the denominator must exclude it");
    }

    /// An ordinary solid is one body and says nothing.
    #[test]
    fn a_plain_solid_is_one_body() {
        let m = two_part(Solid::sphere(40.0), Solid::sphere(10.0));
        let found = disconnected_parts(&m, &cells(3.0));
        assert!(found.split.is_empty(), "got {found:?}");
        assert_eq!(found.examined, 2);
    }
}

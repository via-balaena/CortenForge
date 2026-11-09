//! A `<flexcomp>` made as MuJoCo 3.5.0's `mjCFlexcomp::Make` makes it
//! (`user_flexcomp.cc:102-653`), where the document writes it.
//!
//! Make checks the flexcomp in its order, has its type's maker generate its
//! points and elements (`MakeGrid`, `MakeBox`, `MakeSquare`, or the written
//! ones of a direct type), pins its points, places them by its pose, and then
//! makes a body for each point not pinned and the flex itself, in MuJoCo's own
//! form: a pinned point (every point of a rigid flexcomp) on the body holding
//! the flexcomp, at its offset there; an unpinned one on its own body, at
//! offset 0. The bodies are ordinary bodies from here on, which every check
//! and pass sees, as MuJoCo's are.
//!
//! [`make_flexcomp`] runs in the walk that makes the composites
//! (`composite::expand_composites`), in document order, as MuJoCo makes a
//! flexcomp when its reader meets it; the frames are composed after it, and a
//! frame around a flexcomp moves none of its bodies (MuJoCo gives the
//! flexcomp the frame's body, not the frame, `xml_native_reader.cc:3622-3625`).
//! [`place_flexcomps`] then puts each flexcomp's bodies where it is written.
//!
//! A type this crate does not make (mesh, gmsh) is refused where Make reaches
//! its maker, and so is a box, cylinder or ellipsoid with a count of 1 on an
//! axis, on which MuJoCo's `MakeBox` never ends its loop; a flexcomp made into
//! nothing would turn every reference to it into another, false refusal. A
//! `<plugin>` is refused after the keyframes, the flexcomp made without it.
//! Forms sim-core does not take (a radial dof, interpolated nodes, a vertex
//! body in a moving or turned holding body, a pinned point on a moving one)
//! are made as MuJoCo makes them, and the builder refuses their flex after
//! the keyframes, built until then with every vertex fixed (a `<flex>`'s are
//! refused where the builder finds them, registry `D-FLEX-VERTEX-BODY`).

use std::f64::consts::PI;

use nalgebra::{Vector3, Vector4};

use crate::defaults::DefaultResolver;
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::types::{
    FlexcompDof, FlexcompType, MjcfBody, MjcfCompiler, MjcfFlex, MjcfFlexcomp, MjcfGeom,
    MjcfGeomType, MjcfInertial, MjcfItem, MjcfJoint, MjcfJointType, Prefix,
};

use super::geom::resolve_geom_fromto;
use super::items::{Contents, Element};
use super::mass::MJ_MINVAL;
use super::orientation::{MJ_EPS, normvec, quat_as_written, rot_vec_quat};

/// MuJoCo's `mjMAXVAL` (`mjmodel.h`), where Make starts its bounding box.
const MJ_MAXVAL: f64 = 1e10;

/// A flexcomp as Make made it: its bodies, its flex, and a refusal this
/// crate makes after the keyframes, if any.
#[derive(Debug)]
pub struct MadeFlexcomp {
    /// Its name, which [`place_flexcomps`] checks against the flexcomp it
    /// puts these bodies in place of.
    name: String,
    /// The bodies of its unpinned points, in point order, then its node
    /// bodies.
    bodies: Vec<MjcfBody>,
    /// Its flex.
    flex: MjcfFlex,
    /// What this crate refuses of it, after the keyframes.
    refusal: Option<MjcfError>,
}

/// What [`place_flexcomps`] returns: each made flexcomp's flex, in document
/// order, and the first refusal this crate makes of one, if any.
pub struct PlacedFlexcomps {
    /// The flexes, after the `<deformable>` ones in the model's order.
    pub flexes: Vec<MjcfFlex>,
    /// The first flexcomp's refusal, in document order, with its flex's
    /// position in `flexes`.
    pub refusal: Option<(usize, MjcfError)>,
}

/// `fc`, written in the body `holder`, made as MuJoCo's `Make` makes it: its
/// checks, in Make's order, refuse it, and so does a type this crate does not
/// make; a `<plugin>` comes back as the made flexcomp's refusal, for after the
/// keyframes.
pub fn make_flexcomp(
    fc: &MjcfFlexcomp,
    holder: &str,
    resolver: &DefaultResolver,
    compiler: &MjcfCompiler,
) -> Result<MadeFlexcomp> {
    make(fc, holder, resolver, compiler)
        .map_err(|e| e.with_location_if_unknown(Location::named(ElementKind::Flex, &fc.name)))
}

/// MuJoCo's refusal of a flexcomp, worded as its `Make` words it.
fn refuse(attribute: &str, reason: impl Into<String>) -> MjcfError {
    MjcfError::invalid_value("flexcomp", attribute, reason)
}

/// A made body's name as MuJoCo writes it, into a `char txt[100]` by
/// `sprintf_arr` (`user_flexcomp.cc:550-552`, `:624-626`): its first 99
/// bytes (at a character's boundary here).
fn made_name(name: String) -> String {
    let mut name = name;
    let mut end = name.len().min(99);
    while !name.is_char_boundary(end) {
        end -= 1;
    }
    name.truncate(end);
    name
}

// The steps follow `Make` one for one, so the function reads as its source.
#[allow(clippy::too_many_lines)]
fn make(
    fc: &MjcfFlexcomp,
    holder: &str,
    resolver: &DefaultResolver,
    compiler: &MjcfCompiler,
) -> Result<MadeFlexcomp> {
    let direct = matches!(
        fc.kind,
        FlexcompType::Direct | FlexcompType::Mesh | FlexcompType::Gmsh
    );
    let radius = fc.flex.radius;

    // `:110-147`: the holding body's name, the dim, the count, the spacing,
    // the scale, the mass and inertia box, the orientation.
    if holder.is_empty() {
        return Err(MjcfError::invalid_model("Parent body must have name"));
    }
    if !(1..=3).contains(&fc.dim) {
        return Err(refuse("dim", "Invalid dim, must be between 1 and 3"));
    }
    if fc
        .count
        .iter()
        .any(|&c| c < 1 || (fc.dof == FlexcompDof::Radial && c < 2 && fc.dim == 3))
    {
        return Err(refuse("count", "Count too small"));
    }
    let minspace = 2.0 * radius + fc.flex.margin;
    if !direct && fc.spacing.iter().any(|&s| s < minspace) {
        return Err(refuse(
            "spacing",
            "Spacing must be larger than geometry size",
        ));
    }
    if fc.scale.iter().any(|&s| s < MJ_MINVAL) {
        return Err(refuse("scale", "Scale must be larger than mjMINVAL"));
    }
    if fc.mass < MJ_MINVAL || fc.inertiabox < MJ_MINVAL {
        return Err(refuse(
            if fc.mass < MJ_MINVAL {
                "mass"
            } else {
                "inertiabox"
            },
            "Mass and inertiabox must be larger than mjMINVAL",
        ));
    }
    let quat = quat_as_written(
        fc.quat,
        fc.euler,
        fc.axisangle,
        fc.xyaxes,
        fc.zaxis,
        compiler,
    )
    .map_err(|e| e.on("flexcomp"))?
    .into_inner();
    let quat = [quat.w, quat.i, quat.j, quat.k];

    // `:149-187`: the type's maker, which may set the dim.
    // The makers append to the written `point` and `element`, as MuJoCo's
    // push onto them (a grid that also writes points keeps them first).
    let mut dim = fc.dim;
    let mut flat: Vec<f64> = fc.point.clone();
    let mut element: Vec<i32> = fc.element.clone();
    match fc.kind {
        FlexcompType::Grid | FlexcompType::Circle => {
            make_grid(fc, dim, &mut flat, &mut element)?;
        }
        FlexcompType::Box | FlexcompType::Cylinder | FlexcompType::Ellipsoid => {
            if fc.count.contains(&1) {
                return Err(refuse(
                    "count",
                    format!(
                        "a {} needs a count of at least 2 on each axis (MuJoCo 3.5.0's MakeBox does not end its loop at 1)",
                        fc.kind.word()
                    ),
                ));
            }
            make_box(fc, dim, &mut flat, &mut element);
        }
        FlexcompType::Square | FlexcompType::Disc => {
            dim = 2;
            make_square(fc, &mut flat, &mut element)?;
        }
        FlexcompType::Mesh | FlexcompType::Gmsh => {
            return Err(MjcfError::unsupported(format!(
                "a flexcomp of type {} (ledger L398, L399)",
                fc.kind.word()
            )));
        }
        FlexcompType::Direct => {}
    }

    // `:195-247`: the pins' sizes, the points and elements.
    let pins = &fc.pins;
    let interpolated = matches!(fc.dof, FlexcompDof::Trilinear | FlexcompDof::Quadratic);
    let udim = usize::try_from(dim).unwrap_or(1);
    if !pins.range.len().is_multiple_of(2) {
        return Err(refuse("pin", "Pin range number must be multiple of 2"));
    }
    if !pins.grid.len().is_multiple_of(udim) {
        return Err(refuse("pin", "Pin grid number must be multiple of dim"));
    }
    if !pins.gridrange.len().is_multiple_of(2 * udim) {
        return Err(refuse(
            "pin",
            "Pin grid range number of must be multiple of 2*dim",
        ));
    }
    let grid_pins = !(pins.grid.is_empty() && pins.gridrange.is_empty());
    if fc.kind != FlexcompType::Grid && grid_pins && !interpolated {
        return Err(refuse(
            "pin",
            "Pin grid(range) can only be used with grid or interpolated",
        ));
    }
    if dim == 1 && grid_pins {
        return Err(refuse("pin", "Pin grid(range) cannot be used with dim=1"));
    }
    if flat.is_empty() || element.is_empty() {
        return Err(refuse("point", "Point and element required"));
    }
    if !flat.len().is_multiple_of(3) {
        return Err(refuse("point", "Point size must be a multiple of 3"));
    }
    if !element.len().is_multiple_of(udim + 1) {
        return Err(refuse(
            "element",
            "Element size must be a multiple of dim+1",
        ));
    }
    let mut points: Vec<[f64; 3]> = flat.chunks(3).map(|p| [p[0], p[1], p[2]]).collect();
    let npnt = points.len();
    let npnt_i = i64::try_from(npnt).unwrap_or(i64::MAX);
    for (i, &id) in element.iter().enumerate() {
        if id < 0 || i64::from(id) >= npnt_i {
            return Err(refuse(
                "element",
                format!("element {i} has point id {id}, number of points is {npnt}"),
            ));
        }
    }

    // `:250-267`: a direct type's scale, then the pose.
    // MuJoCo compares the scale with 1 exactly (`:250`).
    #[allow(clippy::float_cmp)]
    let scaled = direct && fc.scale != [1.0; 3];
    if scaled {
        for p in &mut points {
            p[0] *= fc.scale[0];
            p[1] *= fc.scale[1];
            p[2] *= fc.scale[2];
        }
    }
    for p in &mut points {
        let turned = rot_vec_quat(*p, quat);
        *p = [
            turned[0] + fc.pos.x,
            turned[1] + fc.pos.y,
            turned[2] + fc.pos.z,
        ];
    }
    let mut minmax = [
        MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, -MJ_MAXVAL, -MJ_MAXVAL, -MJ_MAXVAL,
    ];
    for p in &points {
        for (j, &x) in p.iter().enumerate() {
            minmax[j] = minmax[j].min(x);
            minmax[j + 3] = minmax[j + 3].max(x);
        }
    }

    // `:278-382`: which points are pinned.
    let nnode = match fc.dof {
        FlexcompDof::Trilinear => 8,
        FlexcompDof::Quadratic => 27,
        FlexcompDof::Full | FlexcompDof::Radial => 0,
    };
    let mut pinned = vec![fc.rigid; npnt.max(nnode)];
    let mut rigid = fc.rigid;
    let mut centered = false;
    if !rigid {
        pin_points(fc, dim, npnt, &mut pinned)?;
        if fc.dof == FlexcompDof::Radial {
            pinned[0] = true;
        }
        let allpin = pinned[..npnt].iter().all(|&p| p);
        let nopin = !pinned[..npnt].iter().any(|&p| p);
        if allpin {
            rigid = true;
        } else if nopin {
            centered = true;
        }
    }

    // `:385-440`: a direct type's unused points dropped.
    if direct {
        let mut used = vec![false; npnt];
        for &id in &element {
            used[usize::try_from(id).unwrap_or(0)] = true;
        }
        if used.iter().any(|&u| !u) {
            let mut reindex = vec![0_i32; npnt];
            let mut next = 0_i32;
            for (i, &u) in used.iter().enumerate() {
                reindex[i] = next;
                if u {
                    next += 1;
                }
            }
            for id in &mut element {
                *id = reindex[usize::try_from(*id).unwrap_or(0)];
            }
            // In place, then cut to `max(new_npnt, nnode)`: the pins past the
            // kept points keep their old values, which a node reads.
            let mut new_npnt = 0;
            for (i, &u) in used.iter().enumerate() {
                if u {
                    points[new_npnt] = points[i];
                    pinned[new_npnt] = pinned[i];
                    new_npnt += 1;
                }
            }
            points.truncate(new_npnt);
            pinned.truncate(new_npnt.max(nnode));
        }
    }
    let npnt = points.len();

    // `:443-466`: the flex; a rigid one is all on the holding body.
    let mut flex = MjcfFlex {
        name: fc.name.clone(),
        dim: udim,
        elements: element
            .chunks(udim + 1)
            .map(|e| {
                e.iter()
                    .map(|&id| usize::try_from(id).unwrap_or(0))
                    .collect()
            })
            .collect(),
        flexcomp_holder: Some(holder.to_string()),
        ..fc.flex.clone()
    };
    let mut made = MadeFlexcomp {
        name: fc.name.clone(),
        bodies: Vec::new(),
        flex: MjcfFlex::default(),
        refusal: fc.plugin.then(|| {
            MjcfError::unsupported("a flexcomp's <plugin> (ledger L339)")
                .at(Location::named(ElementKind::Flex, &fc.name))
        }),
    };
    // A rigid flexcomp returns before Make makes its edge equality
    // (`:463-466`, `:643-649`); one Make takes as not rigid keeps it, rigid as
    // its compiled flex may be (an unused free point dropped, `:384-437`).
    if rigid {
        flex.body = vec![holder.to_string()];
        flex.vertices = points.iter().map(|p| Vector3::from(*p)).collect();
        flex.edge_equality = None;
        made.flex = flex;
        return Ok(made);
    }

    // `:468-571`: a body for each unpinned point.
    // A point count is far below 2^53.
    #[allow(clippy::cast_precision_loss)]
    let npnt_f = npnt as f64;
    let bodymass = fc.mass / npnt_f;
    let bodyinertia = bodymass * (2.0 * fc.inertiabox * fc.inertiabox) / 3.0;
    let mut vert: Vec<Vector3<f64>> = Vec::with_capacity(npnt);
    for (i, p) in points.iter().enumerate() {
        if pinned[i] || interpolated {
            flex.body.push(holder.to_string());
            vert.push(Vector3::from(*p));
            continue;
        }
        let name = made_name(format!("{}_{i}", fc.name));
        let mut body = point_body(&name, *p, bodymass, bodyinertia);
        if fc.flex.vertcollide {
            body.geoms.push(sphere(fc, radius, Some(4), resolver)?);
            body.items.push(MjcfItem::Geom);
        }
        match fc.dof {
            FlexcompDof::Radial => {
                let mut axis = *p;
                normvec(&mut axis);
                body.joints
                    .push(slide(fc, &name, Vector3::from(axis), resolver)?);
                body.items.push(MjcfItem::Joint);
            }
            FlexcompDof::Full => {
                for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
                    body.joints.push(slide(fc, &name, axis, resolver)?);
                    body.items.push(MjcfItem::Joint);
                }
            }
            FlexcompDof::Trilinear | FlexcompDof::Quadratic => {}
        }
        flex.body.push(name);
        vert.push(Vector3::zeros());
        made.bodies.push(body);
    }

    // `:573-637`: the node bodies of an interpolated flexcomp.
    if interpolated {
        let order: u8 = if fc.dof == FlexcompDof::Trilinear {
            1
        } else {
            2
        };
        let step = 1.0 / f64::from(order);
        let mass_p2 = [1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0];
        let mut idx = 0;
        for i in 0..=order {
            for j in 0..=order {
                for k in 0..=order {
                    let pos = [
                        minmax[0] + f64::from(i) * step * (minmax[3] - minmax[0]),
                        minmax[1] + f64::from(j) * step * (minmax[4] - minmax[1]),
                        minmax[2] + f64::from(k) * step * (minmax[5] - minmax[2]),
                    ];
                    if pinned[idx] {
                        flex.node.push(holder.to_string());
                        idx += 1;
                        continue;
                    }
                    let mass = if order == 1 {
                        fc.mass / 8.0
                    } else {
                        fc.mass
                            * mass_p2[usize::from(i)]
                            * mass_p2[usize::from(j)]
                            * mass_p2[usize::from(k)]
                    };
                    let inertia = mass * (2.0 * fc.inertiabox * fc.inertiabox) / 3.0;
                    let name = made_name(format!("{}_{i}_{j}_{k}", fc.name));
                    let mut body = point_body(&name, pos, mass, inertia);
                    if fc.flex.vertcollide {
                        body.geoms.push(sphere(fc, radius, None, resolver)?);
                        body.items.push(MjcfItem::Geom);
                    }
                    for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
                        body.joints.push(slide(fc, &name, axis, resolver)?);
                        body.items.push(MjcfItem::Joint);
                    }
                    flex.node.push(name);
                    made.bodies.push(body);
                    idx += 1;
                }
            }
        }
    }

    // `:639-641`: the offsets, unless every point has a body of its own.
    if !centered || interpolated {
        flex.vertices = vert;
    }
    made.flex = flex;
    Ok(made)
}

/// `MakeGrid` (`user_flexcomp.cc:668-798`): a dim-1 line or circle, a dim-2
/// grid of triangles (two per square, flipped across a disc's axes), or a
/// dim-3 grid of tetrahedra (six per cube), centred on the origin.
fn make_grid(
    fc: &MjcfFlexcomp,
    dim: i32,
    points: &mut Vec<f64>,
    element: &mut Vec<i32>,
) -> Result<()> {
    let [c0, c1, c2] = fc.count;
    let s = fc.spacing;
    // A count's centre, `0.5 * (count - 1)`, in MuJoCo's arithmetic.
    let centre = |c: i32| 0.5 * f64::from(c - 1);
    // MuJoCo's grid ids are C ints; their wrapping is its.
    let lin = |a: i32, b: i32| a.wrapping_add(b);
    if dim == 1 {
        for ix in 0..c0 {
            if fc.kind == FlexcompType::Circle {
                if ix >= c0 - 1 {
                    continue;
                }
                let theta = 2.0 * PI / f64::from(c0 - 1);
                let radius = s[0] / (theta / 2.0).sin() / 2.0;
                points.extend([
                    radius * (theta * f64::from(ix)).cos(),
                    radius * (theta * f64::from(ix)).sin(),
                    0.0,
                ]);
                element.push(ix);
                element.push(if ix == c0 - 2 { 0 } else { ix + 1 });
            } else {
                points.extend([s[0] * (f64::from(ix) - centre(c0)), 0.0, 0.0]);
                if ix < c0 - 1 {
                    element.push(ix);
                    element.push(ix + 1);
                }
            }
        }
    } else if dim == 2 {
        let row = c2.wrapping_mul(c1);
        for ix in 0..c0 {
            for iy in 0..c1 {
                let mut quad2tri = [[0, 1, 2], [0, 2, 3]];
                let pos = [
                    s[0] * (f64::from(ix) - centre(c0)),
                    s[1] * (f64::from(iy) - centre(c1)),
                ];
                points.extend([pos[0], pos[1], 0.0]);
                if ((pos[0] < -MJ_EPS && pos[1] > -MJ_EPS)
                    || (pos[0] > -MJ_EPS && pos[1] < -MJ_EPS))
                    && fc.kind == FlexcompType::Disc
                {
                    quad2tri[0][2] = 3;
                    quad2tri[1][0] = 1;
                }
                if ix < c0 - 1 && iy < c1 - 1 {
                    let vert = [
                        lin(row.wrapping_mul(ix), c2.wrapping_mul(iy)),
                        lin(row.wrapping_mul(ix + 1), c2.wrapping_mul(iy)),
                        lin(row.wrapping_mul(ix + 1), c2.wrapping_mul(iy + 1)),
                        lin(row.wrapping_mul(ix), c2.wrapping_mul(iy + 1)),
                    ];
                    for tri in quad2tri {
                        for v in tri {
                            element.push(vert[v]);
                        }
                    }
                }
            }
        }
    } else {
        const CUBE2TETS: [[usize; 4]; 6] = [
            [0, 3, 1, 7],
            [0, 1, 4, 7],
            [1, 3, 2, 7],
            [1, 2, 6, 7],
            [1, 5, 4, 7],
            [1, 6, 5, 7],
        ];
        let row = c2.wrapping_mul(c1);
        let id = |x: i32, y: i32, z: i32| lin(lin(row.wrapping_mul(x), c2.wrapping_mul(y)), z);
        for ix in 0..c0 {
            for iy in 0..c1 {
                for iz in 0..c2 {
                    points.extend([
                        s[0] * (f64::from(ix) - centre(c0)),
                        s[1] * (f64::from(iy) - centre(c1)),
                        s[2] * (f64::from(iz) - centre(c2)),
                    ]);
                    if ix < c0 - 1 && iy < c1 - 1 && iz < c2 - 1 {
                        let vert = [
                            id(ix, iy, iz),
                            id(ix + 1, iy, iz),
                            id(ix + 1, iy + 1, iz),
                            id(ix, iy + 1, iz),
                            id(ix, iy, iz + 1),
                            id(ix + 1, iy, iz + 1),
                            id(ix + 1, iy + 1, iz + 1),
                            id(ix, iy + 1, iz + 1),
                        ];
                        for tet in CUBE2TETS {
                            for v in tet {
                                element.push(vert[v]);
                            }
                        }
                    }
                }
            }
        }
    }
    if element.is_empty() {
        return Err(refuse("count", "No elements were created in grid"));
    }
    Ok(())
}

/// `MakeSquare` (`user_flexcomp.cc:880-906`): a dim-2 grid, a disc's points
/// then pushed out along their own direction to the disc.
fn make_square(fc: &MjcfFlexcomp, points: &mut Vec<f64>, element: &mut Vec<i32>) -> Result<()> {
    make_grid(fc, 2, points, element)?;
    if fc.kind == FlexcompType::Disc {
        let size = [
            0.5 * fc.spacing[0] * f64::from(fc.count[0] - 1),
            0.5 * fc.spacing[1] * f64::from(fc.count[1] - 1),
        ];
        // Every point, a written one too, as MuJoCo's loop over `point`.
        for p in points.chunks_exact_mut(3) {
            let l0 = p[0].abs().max(p[1].abs());
            let mut xy = [p[0], p[1]];
            normvec(&mut xy);
            // `pos[0] *= size[0]*L0`: the product of the size and L0 first.
            p[0] = xy[0] * (size[0] * l0);
            p[1] = xy[1] * (size[1] * l0);
        }
    }
    Ok(())
}

/// `MakeBox` (`user_flexcomp.cc:917-1110`, `open` true as `Make` calls it):
/// a box's surface points, projected onto a cylinder or an ellipsoid
/// (`BoxProject`, `:838-875`); with dim 3 a centre point first, and each
/// surface triangle with the centre a tetrahedron (`BoxID`, `:803-833`);
/// with dim 2 the triangles, the top face left open. Every count is at
/// least 2 here.
// The loops follow MuJoCo's, side by side.
#[allow(clippy::too_many_lines)]
fn make_box(fc: &MjcfFlexcomp, dim: i32, points: &mut Vec<f64>, element: &mut Vec<i32>) {
    let [c0, c1, c2] = fc.count;
    let open = true;
    let dim2_open = open && dim == 2;
    if dim == 3 {
        points.extend([0.0; 3]);
    }
    let lin3 = |ix: i32, iy: i32, iz: i32| {
        ix.wrapping_mul(c1)
            .wrapping_mul(c2)
            .wrapping_add(iy.wrapping_mul(c2))
            .wrapping_add(iz)
    };
    let cells = usize::try_from(i64::from(c0) * i64::from(c1) * i64::from(c2)).unwrap_or(0);
    let mut idx = vec![0_i32; cells];
    let slot = |l: i32| usize::try_from(l).unwrap_or(0);
    let mut n = 0_i32;
    let project = |ix: i32, iy: i32, iz: i32| box_project(fc, ix, iy, iz);

    // The points: the faces iz = 0 and iz = max, then iy = 0 and max, then
    // ix = 0 and max, each without the rows an earlier face holds.
    let mut iz = 0;
    while iz < c2 {
        for ix in 0..c0 {
            for iy in 0..c1 {
                if dim2_open && iz != 0 {
                    continue;
                }
                points.extend(project(ix, iy, iz));
                idx[slot(lin3(ix, iy, iz))] = n;
                n += 1;
            }
        }
        iz += c2 - 1;
    }
    let mut iy = 0;
    while iy < c1 {
        for ix in 0..c0 {
            for iz in 0..c2 {
                if iz > 0 && (dim2_open || iz < c2 - 1) {
                    points.extend(project(ix, iy, iz));
                    idx[slot(lin3(ix, iy, iz))] = n;
                    n += 1;
                }
            }
        }
        iy += c1 - 1;
    }
    let mut ix = 0;
    while ix < c0 {
        for iy in 0..c1 {
            for iz in 0..c2 {
                if iz > 0 && (dim2_open || iz < c2 - 1) && iy > 0 && iy < c1 - 1 {
                    points.extend(project(ix, iy, iz));
                    idx[slot(lin3(ix, iy, iz))] = n;
                    n += 1;
                }
            }
        }
        ix += c0 - 1;
    }

    let at = |ix: i32, iy: i32, iz: i32| idx[slot(lin3(ix, iy, iz))];
    let bid = |ix: i32, iy: i32, iz: i32| box_id(fc.count, ix, iy, iz);

    // The elements, face by face in the same order.
    let mut iz = 0;
    while iz < c2 {
        for ix in 0..c0 {
            for iy in 0..c1 {
                if dim2_open && iz != 0 {
                    continue;
                }
                if ix < c0 - 1 && iy < c1 - 1 {
                    if dim == 3 {
                        element.extend([
                            0,
                            bid(ix, iy, iz),
                            bid(ix + 1, iy, iz),
                            bid(ix + 1, iy + 1, iz),
                        ]);
                        element.extend([
                            0,
                            bid(ix, iy, iz),
                            bid(ix, iy + 1, iz),
                            bid(ix + 1, iy + 1, iz),
                        ]);
                    } else {
                        let (step1, step2) = if iz == 0 { (1, 0) } else { (0, 1) };
                        element.extend([
                            at(ix, iy, iz),
                            at(ix + 1, iy + step1, iz),
                            at(ix + 1, iy + step2, iz),
                        ]);
                        element.extend([
                            at(ix, iy, iz),
                            at(ix + step2, iy + 1, iz),
                            at(ix + step1, iy + 1, iz),
                        ]);
                    }
                }
            }
        }
        iz += c2 - 1;
    }
    let mut iy = 0;
    while iy < c1 {
        for ix in 0..c0 {
            for iz in 0..c2 {
                if ix < c0 - 1 && iz < c2 - 1 {
                    if dim == 3 {
                        element.extend([
                            0,
                            bid(ix, iy, iz),
                            bid(ix + 1, iy, iz),
                            bid(ix + 1, iy, iz + 1),
                        ]);
                        element.extend([
                            0,
                            bid(ix, iy, iz),
                            bid(ix, iy, iz + 1),
                            bid(ix + 1, iy, iz + 1),
                        ]);
                    } else {
                        let ix0 = if iy == 0 { ix } else { ix + 1 };
                        let dx = if iy == 0 { 1 } else { -1 };
                        element.extend([
                            at(ix0, iy, iz),
                            at(ix0 + dx, iy, iz),
                            at(ix0 + dx, iy, iz + 1),
                        ]);
                        element.extend([
                            at(ix0, iy, iz),
                            at(ix0 + dx, iy, iz + 1),
                            at(ix0, iy, iz + 1),
                        ]);
                    }
                }
            }
        }
        iy += c1 - 1;
    }
    let mut ix = 0;
    while ix < c0 {
        for iy in 0..c1 {
            for iz in 0..c2 {
                if iy < c1 - 1 && iz < c2 - 1 {
                    if dim == 3 {
                        element.extend([
                            0,
                            bid(ix, iy, iz),
                            bid(ix, iy + 1, iz),
                            bid(ix, iy + 1, iz + 1),
                        ]);
                        element.extend([
                            0,
                            bid(ix, iy, iz),
                            bid(ix, iy, iz + 1),
                            bid(ix, iy + 1, iz + 1),
                        ]);
                    } else {
                        let iy0 = if ix == 0 { iy + 1 } else { iy };
                        let dy = if ix == 0 { -1 } else { 1 };
                        element.extend([
                            at(ix, iy0, iz),
                            at(ix, iy0 + dy, iz),
                            at(ix, iy0 + dy, iz + 1),
                        ]);
                        element.extend([
                            at(ix, iy0, iz),
                            at(ix, iy0 + dy, iz + 1),
                            at(ix, iy0, iz + 1),
                        ]);
                    }
                }
            }
        }
        ix += c0 - 1;
    }
}

/// `BoxID` (`user_flexcomp.cc:803-833`): a box surface point's id, after the
/// centre point.
fn box_id(count: [i32; 3], ix: i32, iy: i32, iz: i32) -> i32 {
    let [c0, c1, c2] = count;
    let face = c0.wrapping_mul(c1);
    let side = c2 - 2;
    if iz == 0 {
        ix.wrapping_mul(c1) + iy + 1
    } else if iz == c2 - 1 {
        face + ix.wrapping_mul(c1) + iy + 1
    } else if iy == 0 {
        2 * face + ix.wrapping_mul(side) + iz - 1 + 1
    } else if iy == c1 - 1 {
        2 * face + c0.wrapping_mul(side) + ix.wrapping_mul(side) + iz - 1 + 1
    } else if ix == 0 {
        2 * face + 2 * c0.wrapping_mul(side) + (iy - 1).wrapping_mul(side) + iz - 1 + 1
    } else {
        2 * face
            + 2 * c0.wrapping_mul(side)
            + (c1 - 2).wrapping_mul(side)
            + (iy - 1).wrapping_mul(side)
            + iz
            - 1
            + 1
    }
}

/// `BoxProject` (`user_flexcomp.cc:838-875`): a box grid point, in [-1, 1]
/// per axis, scaled to the box, or projected onto the cylinder or the
/// ellipsoid of the box's half sizes.
fn box_project(fc: &MjcfFlexcomp, ix: i32, iy: i32, iz: i32) -> [f64; 3] {
    let [c0, c1, c2] = fc.count;
    let mut pos = [
        2.0 * f64::from(ix) / f64::from(c0 - 1) - 1.0,
        2.0 * f64::from(iy) / f64::from(c1 - 1) - 1.0,
        2.0 * f64::from(iz) / f64::from(c2 - 1) - 1.0,
    ];
    let size = [
        0.5 * fc.spacing[0] * f64::from(c0 - 1),
        0.5 * fc.spacing[1] * f64::from(c1 - 1),
        0.5 * fc.spacing[2] * f64::from(c2 - 1),
    ];
    match fc.kind {
        FlexcompType::Box => {
            pos[0] *= size[0];
            pos[1] *= size[1];
            pos[2] *= size[2];
        }
        FlexcompType::Cylinder => {
            let l0 = pos[0].abs().max(pos[1].abs());
            let mut xy = [pos[0], pos[1]];
            normvec(&mut xy);
            pos[0] = xy[0] * (size[0] * l0);
            pos[1] = xy[1] * (size[1] * l0);
            pos[2] *= size[2];
        }
        FlexcompType::Ellipsoid => {
            normvec(&mut pos);
            pos[0] *= size[0];
            pos[1] *= size[1];
            pos[2] *= size[2];
        }
        _ => {}
    }
    pos
}

/// The pins of `<pin>` (`user_flexcomp.cc:285-356`), in MuJoCo's order: ids,
/// ranges, grid points, grid ranges, each refused where it is out of range.
fn pin_points(fc: &MjcfFlexcomp, dim: i32, npnt: usize, pinned: &mut [bool]) -> Result<()> {
    let pins = &fc.pins;
    let in_points = |id: i32| usize::try_from(id).ok().filter(|&i| i < npnt);
    for &id in &pins.id {
        let i = in_points(id).ok_or_else(|| refuse("pin", "pinid out of range"))?;
        pinned[i] = true;
    }
    for pair in pins.range.chunks(2) {
        let (Some(a), Some(b)) = (in_points(pair[0]), in_points(pair[1])) else {
            return Err(refuse("pin", "pinrange out of range"));
        };
        for p in pinned.iter_mut().take(b + 1).skip(a) {
            *p = true;
        }
    }
    let count = fc.count;
    // A grid id beyond the points marks nothing: it is reached only by an
    // interpolated flexcomp of another type, which the builder refuses.
    let mut mark = |id: i32| {
        if let Some(p) = usize::try_from(id).ok().and_then(|i| pinned.get_mut(i)) {
            *p = true;
        }
    };
    let udim = usize::try_from(dim).unwrap_or(1);
    for g in pins.grid.chunks(udim) {
        let check = if fc.kind != FlexcompType::Grid
            && matches!(fc.dof, FlexcompDof::Trilinear | FlexcompDof::Quadratic)
        {
            [if fc.dof == FlexcompDof::Trilinear {
                2
            } else {
                3
            }; 3]
        } else {
            count
        };
        if g.iter().zip(check).any(|(&v, c)| v < 0 || v >= c) {
            return Err(refuse("pin", "pingrid out of range"));
        }
        if dim == 2 {
            mark(g[0].wrapping_mul(count[1]).wrapping_add(g[1]));
        } else if dim == 3 {
            mark(grid_id3(count, g[0], g[1], g[2]));
        }
    }
    for r in pins.gridrange.chunks(2 * udim) {
        if r.iter()
            .enumerate()
            .any(|(k, &v)| v < 0 || v >= count[k % udim])
        {
            return Err(refuse("pin", "pingridrange out of range"));
        }
        if dim == 2 {
            for ix in r[0]..=r[2] {
                for iy in r[1]..=r[3] {
                    mark(ix.wrapping_mul(count[1]).wrapping_add(iy));
                }
            }
        } else if dim == 3 {
            for ix in r[0]..=r[3] {
                for iy in r[1]..=r[4] {
                    for iz in r[2]..=r[5] {
                        mark(grid_id3(count, ix, iy, iz));
                    }
                }
            }
        }
    }
    Ok(())
}

/// `GridID(ix, iy, iz)` (`user_flexcomp.cc:660-662`).
fn grid_id3(count: [i32; 3], ix: i32, iy: i32, iz: i32) -> i32 {
    ix.wrapping_mul(count[1])
        .wrapping_mul(count[2])
        .wrapping_add(iy.wrapping_mul(count[2]))
        .wrapping_add(iz)
}

/// A point's body (`:498-521`): at the point, with its mass and inertia
/// written, at its origin.
fn point_body(name: &str, pos: [f64; 3], mass: f64, inertia: f64) -> MjcfBody {
    MjcfBody {
        name: name.to_string(),
        pos: Vector3::from(pos),
        inertial: Some(MjcfInertial {
            pos: Vector3::zeros(),
            quat: Vector4::new(1.0, 0.0, 0.0, 0.0),
            mass,
            diaginertia: Some(Vector3::repeat(inertia)),
            ..MjcfInertial::default()
        }),
        ..MjcfBody::default()
    }
}

/// A slide of a point's body along `axis` (`:524-546`), at its origin and
/// otherwise its class's, as `mjs_addJoint(pb, 0)` makes it.
fn slide(
    fc: &MjcfFlexcomp,
    body: &str,
    axis: Vector3<f64>,
    resolver: &DefaultResolver,
) -> Result<MjcfJoint> {
    let joint = MjcfJoint {
        class: fc.class.clone(),
        joint_type: Some(MjcfJointType::Slide),
        pos: Some(Vector3::zeros()),
        axis: Some(axis),
        body: Some(body.to_string()),
        ..MjcfJoint::default()
    };
    resolver.apply_to_joint(&joint)
}

/// A point's sphere (`:503-509`): the flex's radius, `group` 4 for a vertex
/// body's (a node body's keeps its class's), otherwise its class's, its
/// `fromto` (a class's) then made into its pose or refused, as every geom's.
fn sphere(
    fc: &MjcfFlexcomp,
    radius: f64,
    group: Option<i32>,
    resolver: &DefaultResolver,
) -> Result<MjcfGeom> {
    let geom = MjcfGeom {
        class: fc.class.clone(),
        geom_type: Some(MjcfGeomType::Sphere),
        size: Prefix::new(&[radius]),
        group,
        ..MjcfGeom::default()
    };
    let mut geom = resolver.apply_to_geom(&geom)?;
    resolve_geom_fromto(&mut geom)?;
    Ok(geom)
}

/// Each flexcomp of `world`'s tree put where it is written: replaced by the
/// bodies [`make_flexcomp`] made of it, in the same document order it made
/// them in (frames have been composed, and kept each flexcomp in its place).
pub fn place_flexcomps(world: &mut MjcfBody, made: Vec<MadeFlexcomp>) -> PlacedFlexcomps {
    let mut made = made.into_iter();
    let mut placed = PlacedFlexcomps {
        flexes: Vec::new(),
        refusal: None,
    };
    place_in(world, &mut made, &mut placed);
    let unplaced = made.count();
    debug_assert_eq!(unplaced, 0, "a made flexcomp was not placed");
    placed
}

/// `body`'s flexcomps placed, then each child body's, in document order.
fn place_in(
    body: &mut MjcfBody,
    made: &mut std::vec::IntoIter<MadeFlexcomp>,
    placed: &mut PlacedFlexcomps,
) {
    let contents = Contents::take_from_body(body);
    let mut kept = Contents::default();
    for element in contents.into_elements() {
        match element {
            Element::Body(mut child) => {
                place_in(&mut child, made, placed);
                kept.push(Element::Body(child));
            }
            Element::Flexcomp(fc) => {
                let Some(one) = made.next() else {
                    debug_assert!(false, "flexcomp '{}' was not made", fc.name);
                    continue;
                };
                debug_assert_eq!(one.name, fc.name, "flexcomps placed out of order");
                for made_body in one.bodies {
                    kept.push(Element::Body(made_body));
                }
                if placed.refusal.is_none() {
                    placed.refusal = one.refusal.map(|e| (placed.flexes.len(), e));
                }
                placed.flexes.push(one.flex);
            }
            other => kept.push(other),
        }
    }
    kept.put_into_body(body);
}

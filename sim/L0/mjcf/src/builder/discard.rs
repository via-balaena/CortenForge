//! `discardvisual` as MuJoCo 3.5.0 makes it (book Rigid-loading L39; ledger
//! L336, L342): every geom and mesh is built and checked, and each body's
//! inertia comes from all its geoms, before the visual geoms and meshes are
//! deleted and every reference to one renumbered (`mjCModel::IndexAssets`
//! with `discard`, `user_model.cc:1987-2023`, once every element has
//! compiled, `:4976`).
//!
//! A geom is visual where its `contype` and `conaffinity` are both 0
//! (`user_objects.cc:3672`) and nothing MuJoCo marks not visual names it: a
//! contact pair (`:5675-5676`), a spatial tendon's geom wrap (`:6396`), a
//! sensor's object or reference (`:7110`, `:7139`), a tuple (`:7918`, which
//! this crate does not read; registry `D-DISCARD-TUPLE`). This crate's
//! distance equality, which MuJoCo refuses, keeps its geoms too. A mesh is
//! visual where no geom that is not visual names it (`:1893-1895`), a
//! primitive fitted to it among them.

use std::collections::{HashMap, HashSet};

use sim_core::{EqualityType, MjObjectType, WrapType};

use super::ModelBuilder;
use crate::types::{MjcfGeom, MjcfModel, MjcfSensorType, SpatialPathElement};

/// The names of the geoms MuJoCo marks not visual, so that `discardvisual`
/// keeps them: those a contact pair, a spatial tendon's geom wrap, a sensor's
/// geom object or reference, or a distance equality names. A frame sensor
/// whose `objtype` is not written, which MuJoCo refuses and this crate reads
/// by its name, keeps a geom of that name. With them, the names of the
/// meshes a geom that is not visual names.
pub struct Referenced {
    geoms: HashSet<String>,
    meshes: HashSet<String>,
}

impl Referenced {
    /// The names `mjcf` references.
    pub fn of(mjcf: &MjcfModel) -> Self {
        let mut names = HashSet::new();
        for pair in &mjcf.contact.pairs {
            names.insert(pair.geom1.clone());
            names.insert(pair.geom2.clone());
        }
        for tendon in &mjcf.tendons {
            for element in &tendon.path_elements {
                if let SpatialPathElement::Geom { geom, .. } = element {
                    names.insert(geom.clone());
                }
            }
        }
        for sensor in &mjcf.sensors {
            let frame = matches!(
                sensor.sensor_type,
                MjcfSensorType::Framepos
                    | MjcfSensorType::Framequat
                    | MjcfSensorType::Framexaxis
                    | MjcfSensorType::Frameyaxis
                    | MjcfSensorType::Framezaxis
                    | MjcfSensorType::Framelinvel
                    | MjcfSensorType::Frameangvel
                    | MjcfSensorType::Framelinacc
                    | MjcfSensorType::Frameangacc
            );
            let object = sensor.objtype.as_deref();
            if object == Some("geom") || (frame && object.is_none()) {
                names.extend(sensor.objname.clone());
            }
            if sensor.reftype.as_deref() == Some("geom") {
                names.extend(sensor.refname.clone());
            }
            names.extend(sensor.geom1.clone());
            names.extend(sensor.geom2.clone());
        }
        for distance in &mjcf.equality.distances {
            names.insert(distance.geom1.clone());
            names.extend(distance.geom2.clone());
        }
        let mut referenced = Self {
            geoms: names,
            meshes: HashSet::new(),
        };
        let mut meshes = HashSet::new();
        let mut bodies = vec![&mjcf.worldbody];
        while let Some(body) = bodies.pop() {
            for geom in &body.geoms {
                if let Some(mesh) = geom.mesh.as_deref().filter(|m| !m.is_empty())
                    && !referenced.geom_is_visual(geom)
                {
                    meshes.insert(mesh.to_string());
                }
            }
            bodies.extend(&body.children);
        }
        referenced.meshes = meshes;
        referenced
    }

    /// Whether `discardvisual` keeps the mesh `name`: a geom that is not
    /// visual names it.
    pub fn keeps_mesh(&self, name: &str) -> bool {
        self.meshes.contains(name)
    }

    /// Whether a geom of this resolved `contype` and `conaffinity` and this
    /// name is visual.
    fn is_visual(&self, contype: u32, conaffinity: u32, name: Option<&str>) -> bool {
        contype == 0 && conaffinity == 0 && !name.is_some_and(|n| self.geoms.contains(n))
    }

    /// Whether the tree's `geom` is visual, its `contype` and `conaffinity`
    /// as its class resolved them (1 where neither writes one), as the
    /// builder keeps them.
    pub fn geom_is_visual(&self, geom: &MjcfGeom) -> bool {
        // As `process_geom` keeps them: a negative mask is not 0 either way.
        #[allow(clippy::cast_sign_loss)]
        let mask = |v: Option<i32>| v.unwrap_or(1) as u32;
        self.is_visual(
            mask(geom.contype),
            mask(geom.conaffinity),
            geom.name.as_deref(),
        )
    }
}

/// Each kept element's index after the discard, `None` for a discarded one
/// (`DeleteElements`, `user_model.cc:1744-1780`: the kept keep their order).
fn renumbered(keep: &[bool]) -> Vec<Option<usize>> {
    let mut next = 0;
    keep.iter()
        .map(|&k| {
            k.then(|| {
                next += 1;
                next - 1
            })
        })
        .collect()
}

/// `v` without the entries `keep` does not keep.
fn retain<T>(v: &mut Vec<T>, keep: &[bool]) {
    debug_assert_eq!(v.len(), keep.len());
    let mut kept = keep.iter();
    v.retain(|_| *kept.next().unwrap_or(&true));
}

impl ModelBuilder {
    /// Delete the visual geoms and the meshes no kept geom names, as
    /// MuJoCo's `IndexAssets` does under `discardvisual`, once every element
    /// has been built and checked: each body keeps the mass, inertia and
    /// inertial frame its geoms gave it, the visual ones among them, and
    /// every geom and mesh reference is renumbered.
    pub(crate) fn discard_visual(&mut self, referenced: &Referenced) {
        let keep: Vec<bool> = (0..self.geom_type.len())
            .map(|g| {
                !referenced.is_visual(
                    self.geom_contype[g],
                    self.geom_conaffinity[g],
                    self.geom_name[g].as_deref(),
                )
            })
            .collect();
        let mesh_keep: Vec<bool> = self
            .mesh_name
            .iter()
            .map(|name| referenced.keeps_mesh(name))
            .collect();
        let geom_id = renumbered(&keep);
        let mesh_id = renumbered(&mesh_keep);
        // A reference to a geom MuJoCo marks not visual, which is kept.
        let kept_geom = |g: usize| geom_id[g].unwrap_or(g);

        // Each body's geoms, contiguous in the builder's order.
        let kept_before = |end: usize| keep[..end].iter().filter(|&&k| k).count();
        for b in 0..self.body_geom_adr.len() {
            let (adr, num) = (self.body_geom_adr[b], self.body_geom_num[b]);
            self.body_geom_adr[b] = kept_before(adr);
            self.body_geom_num[b] = kept_before(adr + num) - kept_before(adr);
        }

        retain(&mut self.geom_type, &keep);
        retain(&mut self.geom_body, &keep);
        retain(&mut self.geom_pos, &keep);
        retain(&mut self.geom_quat, &keep);
        retain(&mut self.geom_size, &keep);
        retain(&mut self.geom_friction, &keep);
        retain(&mut self.geom_condim, &keep);
        retain(&mut self.geom_shellinertia_on_mesh, &keep);
        retain(&mut self.geom_sdf_names_no_mesh, &keep);
        retain(&mut self.geom_contype, &keep);
        retain(&mut self.geom_conaffinity, &keep);
        retain(&mut self.geom_name, &keep);
        retain(&mut self.geom_mesh, &keep);
        retain(&mut self.geom_solref, &keep);
        retain(&mut self.geom_solimp, &keep);
        retain(&mut self.geom_priority, &keep);
        retain(&mut self.geom_solmix, &keep);
        retain(&mut self.geom_margin, &keep);
        retain(&mut self.geom_gap, &keep);
        retain(&mut self.geom_group, &keep);
        retain(&mut self.geom_rgba, &keep);
        retain(&mut self.geom_fluid, &keep);
        retain(&mut self.geom_hfield, &keep);
        retain(&mut self.geom_sdf, &keep);
        retain(&mut self.geom_user_raw, &keep);
        if let Some(turns) = self.compile_turns.as_mut() {
            retain(&mut turns.geom, &keep);
            retain(&mut turns.geom_index, &keep);
        }
        self.geom_name_to_id = self
            .geom_name
            .iter()
            .enumerate()
            .filter_map(|(g, name)| Some((name.clone().filter(|n| !n.is_empty())?, g)))
            .collect::<HashMap<_, _>>();
        for mesh in &mut self.geom_mesh {
            *mesh = mesh.and_then(|m| mesh_id[m]);
        }

        retain(&mut self.mesh_name, &mesh_keep);
        retain(&mut self.mesh_data, &mesh_keep);
        retain(&mut self.mesh_props, &mesh_keep);
        self.mesh_name_to_id = self
            .mesh_name
            .iter()
            .enumerate()
            .map(|(m, name)| (name.clone(), m))
            .collect();

        for (w, &kind) in self.wrap_type.iter().enumerate() {
            if kind == WrapType::Geom {
                self.wrap_objid[w] = kept_geom(self.wrap_objid[w]);
            }
        }
        for s in 0..self.sensor_objid.len() {
            if self.sensor_objtype[s] == MjObjectType::Geom {
                self.sensor_objid[s] = kept_geom(self.sensor_objid[s]);
            }
            if self.sensor_reftype[s] == MjObjectType::Geom {
                self.sensor_refid[s] = kept_geom(self.sensor_refid[s]);
            }
        }
        for pair in &mut self.contact_pairs {
            pair.geom1 = kept_geom(pair.geom1);
            pair.geom2 = kept_geom(pair.geom2);
        }
        self.contact_pair_set = self
            .contact_pair_set
            .iter()
            .map(|&(a, b)| (kept_geom(a), kept_geom(b)))
            .collect();
        for e in 0..self.eq_type.len() {
            if self.eq_type[e] == EqualityType::Distance {
                for id in [&mut self.eq_obj1id[e], &mut self.eq_obj2id[e]] {
                    if *id < keep.len() {
                        *id = kept_geom(*id);
                    }
                }
            }
        }
    }
}

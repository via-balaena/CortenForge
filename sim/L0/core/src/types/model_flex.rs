//! Each flex's element layers and element–vertex pairs, as MuJoCo's compiler
//! makes them from its elements (`mjCFlex::CreateShellPair`,
//! `user_mesh.cc:4473-4671`): the elements that share a fragment (a cable's
//! vertex, a cloth's edge, a tetrahedron's face), those on the border (a
//! fragment no other element has), each element's distance from the border,
//! and the pairs internal collision tests.

use super::model::Model;

impl Model {
    /// Each flex's element layers (`flex_elemlayer`) and element–vertex pairs
    /// (`flex_evpair`, `flex_evpairadr`, `flex_evpairnum`), from `flex_dim`,
    /// `flex_vertadr`, `flex_elemadr`, `flex_elemnum` and `flexelem_data`,
    /// as MuJoCo's compiler (`user_mesh.cc:4473-4671`).
    ///
    /// A flex of dim below 3 has every element on layer 0; a tetrahedron's
    /// layer is its distance, in tetrahedra sharing a face, from the nearest
    /// one with a face on the border. A flex of dim 3 has no pairs.
    pub fn compute_flex_topology(&mut self) {
        self.flex_elemlayer = vec![0; self.nflexelem];
        self.flex_evpair.clear();
        self.flex_evpairadr = vec![0; self.nflex];
        self.flex_evpairnum = vec![0; self.nflex];
        for f in 0..self.nflex {
            let elements: Vec<Vec<usize>> = (0..self.flex_elemnum[f])
                .map(|e| {
                    let global = self.flex_elemadr[f] + e;
                    let adr = self.flexelem_dataadr[global];
                    self.flexelem_data[adr..adr + self.flexelem_datanum[global]]
                        .iter()
                        .map(|&v| v - self.flex_vertadr[f])
                        .collect()
                })
                .collect();
            let (layer, pairs) = layers_and_pairs(self.flex_dim[f], &elements);
            let adr = self.flex_elemadr[f];
            self.flex_elemlayer[adr..adr + layer.len()].copy_from_slice(&layer);
            self.flex_evpairadr[f] = self.flex_evpair.len();
            self.flex_evpairnum[f] = pairs.len();
            self.flex_evpair.extend(pairs);
        }
    }

    /// Whether element `e` of flex `f` (flex-local) collides with the flex's
    /// other elements and enters its hierarchy (`mj_isElemActive`,
    /// `engine_collision_driver.c:214-220`): every element of a flex of dim
    /// below 3, a tetrahedron while its layer (`flex_elemlayer`) is below
    /// `flex_activelayers`.
    pub(crate) fn flex_elem_active(&self, f: usize, e: usize) -> bool {
        self.flex_dim[f] < 3
            || usize::try_from(self.flex_activelayers[f])
                .is_ok_and(|layers| self.flex_elemlayer[self.flex_elemadr[f] + e] < layers)
    }
}

/// One flex's element layers and element–vertex pairs, its elements' vertices
/// numbered within the flex.
fn layers_and_pairs(dim: usize, elements: &[Vec<usize>]) -> (Vec<usize>, Vec<[usize; 2]>) {
    let nelem = elements.len();
    if nelem == 0 {
        return (Vec::new(), Vec::new());
    }

    // Each element's fragments, MuJoCo's (`:4479-4563`): the fragment's
    // vertices sorted, then the element; then every fragment sorted. (MuJoCo
    // keeps the vertices as listed too, for the shell it draws; the element
    // already tells any two entries apart, so they order nothing.)
    let fragments: &[&[usize]] = match dim {
        1 => &[&[0], &[1]],
        2 => &[&[0, 1], &[2, 0], &[1, 2]],
        _ => &[&[0, 1, 2], &[0, 3, 1], &[0, 2, 3], &[1, 3, 2]],
    };
    let mut spec: Vec<Vec<usize>> = Vec::with_capacity(nelem * fragments.len());
    for (e, element) in elements.iter().enumerate() {
        for fragment in fragments {
            let mut entry: Vec<usize> = fragment.iter().map(|&i| element[i]).collect();
            entry.sort_unstable();
            entry.push(e);
            spec.push(entry);
        }
    }
    spec.sort();

    // Two elements with the same fragment share it; an element with a
    // fragment no other has is on the border (`:4565-4603`).
    let mut border = vec![false; nelem];
    let mut shared: Vec<(usize, usize, &[usize])> = Vec::new();
    let mut count = 1;
    for n in 1..spec.len() {
        if spec[n - 1][..dim] == spec[n][..dim] {
            shared.push((spec[n - 1][dim], spec[n][dim], &spec[n][..dim]));
            count += 1;
        } else {
            if count == 1 {
                border[spec[n - 1][dim]] = true;
            }
            count = 1;
        }
    }
    if count == 1 {
        border[spec[spec.len() - 1][dim]] = true;
    }

    // Layers by value iteration over the shared fragments (`:4613-4642`).
    let mut layer = vec![0; nelem];
    if dim == 3 {
        layer = border
            .iter()
            .map(|&b| if b { 0 } else { nelem + 1 })
            .collect();
        let mut change = true;
        while change {
            change = false;
            for &(e1, e2, _) in &shared {
                if layer[e1] > layer[e2] + 1 {
                    layer[e1] = layer[e2] + 1;
                    change = true;
                } else if layer[e2] > layer[e1] + 1 {
                    layer[e2] = layer[e1] + 1;
                    change = true;
                }
            }
        }
    }

    // Where two elements sharing a fragment have one on the border, each
    // one's first vertex off the fragment, with the other (`:4644-4670`).
    let mut pairs = Vec::new();
    if dim < 3 {
        for &(e1, e2, fragment) in &shared {
            if border[e1] || border[e2] {
                for (this, other) in [(e1, e2), (e2, e1)] {
                    if let Some(&v) = elements[this].iter().find(|v| !fragment.contains(v)) {
                        pairs.push([other, v]);
                    }
                }
            }
        }
    }
    (layer, pairs)
}

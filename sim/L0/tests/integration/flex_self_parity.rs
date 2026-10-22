//! A flex against a flex, against MuJoCo 3.5.0.
//!
//! MuJoCo pairs two flexes in its broadphase, descends their hierarchies
//! together, each pair of leaves to `mj_collideElems`, and sorts the pair's
//! contacts by element; without the midphase, every element of the first
//! against every element of the second. Two cables' elements meet as
//! capsules, any other two through the convex route
//! (`engine_collision_driver.c:264-482`, `:622-847`, `:2002-2098`). The
//! golden is `assets/golden/flex/flex_self.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_flex_self_reference.py`, which describes its
//! cases).

use serde_json::Value;

use super::flex_golden::differences;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_self.json"))
        .expect("flex self golden parses")
}

/// Each case of the golden as [`differences`] compares it.
#[test]
fn flexes_collide_with_each_other_as_mujoco() {
    let off = differences(&golden());
    assert!(
        off.is_empty(),
        "{} values differ:\n{}",
        off.len(),
        off.join("\n")
    );
}

//! The bodies' `sleep` policies, checked and set as MuJoCo 3.5.0's compiler
//! checks and sets them: a policy other than `auto` on a moving tree's first
//! body only, as `CopyTree` numbers the trees (`user_model.cc:3036-3048`); an
//! explicit `allowed` or `init` on no tree a coupling tendon reaches, as
//! `mj_setConst` resolves the automatic policies (`engine_setconst.c:203-246`);
//! and, with sleep enabled, every `init` tree put to sleep, as the compiler's
//! final `mj_makeData` puts them (`user_model.cc:5159-5185`).

use sim_core::{MakeDataError, Model, SleepPolicy};

use crate::error::{ElementKind, Location, MjcfError, Result};

/// Set each body's explicit policy on its tree, refusing a policy other than
/// `auto` on a body that is not the first body of a moving tree: one welded to
/// the world (a static or mocap body) or one after its tree's first body.
/// MuJoCo tells the first body by the body before it, in another tree or in
/// none (`user_model.cc:3043-3046`), and so does this. The bodies are
/// checked in order; a body `fusestatic` fused is not checked, as MuJoCo
/// fuses before it numbers the trees. Run on the built model, after the
/// checks MuJoCo makes before `CopyTree` numbers the trees and before the
/// keyframes, which MuJoCo compiles after it (`:5021-5028`).
///
/// # Errors
/// "sleep policy only allowed for movable root bodies" at the first such
/// body, as MuJoCo's.
pub fn apply_body_sleep_policies(
    model: &mut Model,
    body_sleep_policy: &[Option<SleepPolicy>],
) -> Result<()> {
    for (body, policy) in body_sleep_policy.iter().enumerate().skip(1) {
        let Some(policy) = *policy else { continue };
        if policy == SleepPolicy::Auto {
            continue;
        }
        let tree = model.body_treeid[body];
        if tree >= model.ntree || model.body_treeid[body - 1] == tree {
            return Err(MjcfError::invalid_model(
                "sleep policy only allowed for movable root bodies",
            )
            .at(Location::nth(
                ElementKind::Body,
                model.body_name[body].as_deref(),
                body,
            )));
        }
        model.tree_sleep_policy[tree] = policy;
    }
    Ok(())
}

/// Refuse an explicit `allowed` or `init` on a tree that a tendon reaching
/// more than two trees, or two with a stiffness or a damping other than 0,
/// reaches, as `mj_setConst` refuses it (`engine_setconst.c:203-246`): the
/// tendons in order, each one's trees in the order of its wraps, the first
/// such tree named; an explicit `never` is not refused. MuJoCo makes this check
/// whether or not sleep is enabled, and after the keyframes
/// (`user_model.cc:5028`, `:5126`).
///
/// # Errors
/// MuJoCo's message, with the tree's and the tendon's ids.
pub fn check_tendon_sleep_policies(model: &Model) -> Result<()> {
    for tendon in 0..model.ntendon {
        let treenum = model.tendon_treenum[tendon];
        let passive = model.tendon_stiffness[tendon] != 0.0 || model.tendon_damping[tendon] != 0.0;
        if treenum < 2 || (treenum == 2 && !passive) {
            continue;
        }
        let flagged = model.tendon_trees(tendon).find(|&tree| {
            matches!(
                model.tree_sleep_policy[tree],
                SleepPolicy::Allowed | SleepPolicy::Init
            )
        });
        if let Some(tree) = flagged {
            let why = if treenum > 2 {
                "which spans more than 2 trees"
            } else {
                "with non-zero stiffness or damping"
            };
            return Err(MjcfError::invalid_model(format!(
                "tree {tree} connected to tendon {tendon} {why}, sleeping not allowed"
            )));
        }
    }
    Ok(())
}

/// With sleep enabled and a tree whose policy is `init`, refuse the model
/// [`Model::try_make_data`] refuses for those trees: one that cannot be put to
/// sleep ([`MakeDataError::InitSleep`]) or whose forward pass before it fails
/// ([`MakeDataError::InitForward`]), as MuJoCo's compiler makes a `mjData`
/// for them, last (`user_model.cc:5159-5185`). A model `try_make_data`
/// refuses for another reason first is not refused here, so its `init` trees
/// are not put to sleep at load.
///
/// # Errors
/// For trees that cannot be put to sleep, MuJoCo's message
/// (`engine_io.c:1490-1492`) on one line, with the root body's name and id;
/// for the forward pass, [`MakeDataError::InitForward`]'s.
pub fn check_init_sleep(model: &Model) -> Result<()> {
    if model.enableflags & sim_core::ENABLE_SLEEP == 0
        || !model.tree_sleep_policy.contains(&SleepPolicy::Init)
    {
        return Ok(());
    }
    match model.try_make_data() {
        Err(MakeDataError::InitSleep {
            marked,
            slept,
            root_body,
            ..
        }) => {
            let name = model.body_name[root_body].as_deref().unwrap_or("");
            Err(MjcfError::invalid_model(format!(
                "{marked} trees were marked as sleep='init' but only {slept} could be slept. \
                 Body '{name}' (id={root_body}) is the root of the first tree that could not be \
                 slept."
            )))
        }
        Err(e @ MakeDataError::InitForward(_)) => Err(MjcfError::invalid_model(e.to_string())),
        _ => Ok(()),
    }
}

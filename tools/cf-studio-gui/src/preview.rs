//! Step 3's live plug preview: the shaped piece, meshed off the main thread.
//!
//! ⚠⚠ The re-mesh is a background job rather than the pre-port's inline call,
//! and a measurement is why. [`PlugPreview::mesh`] samples a mesh-BVH-backed
//! SDF, so its cost tracks the *scan's* triangle count rather than the preview
//! grid: 97 ms on a 51 k-triangle scan, 191 ms on a 241 k one, against a 16 ms
//! frame. Slint called it inline from `shape-changed()` and froze for the
//! duration; here that is six to twelve dropped frames on every `+` click.
//!
//! ⚠ Which control moved is not worth knowing. Ridges on cost 97.6 ms against
//! 97.0 off — the cost is the sampling, not the field — so the driver compares
//! the whole [`PlugDraft`] and re-meshes all of it.

use std::panic::AssertUnwindSafe;
use std::sync::Arc;

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use cf_studio_core::{PlugDraft, PrepInput, Step};
use cf_studio_engine::{PlugPreview, proxy_preview_mesh};
use mesh_types::IndexedMesh;

use crate::scan::ScanEdit;
use crate::shape::ShapeControls;
use crate::state::Studio;

/// The cleaned scan behind the preview.
///
/// Both end states are final, which is what lets [`PlugView::showing_proxy`]
/// read the note's answer off this instead of tracking it per mesh: a `Ready`
/// cache never fails later, and an `Unavailable` one is never retried.
#[derive(Default)]
enum Cache {
    /// Nothing built, and nothing building it.
    #[default]
    Cold,
    /// The flood-fill SDF is being built — hundreds of milliseconds, so it runs
    /// once per session rather than once per edit.
    Building(Task<Option<PlugPreview>>),
    /// Ready: the preview is this body.
    Ready(Arc<PlugPreview>),
    /// It could not be built. The preview falls back to the generic proxy, and
    /// the screen says so.
    Unavailable,
}

/// A re-mesh in flight, and the draft it will answer for.
struct Meshing {
    draft: PlugDraft,
    task: Task<Option<IndexedMesh>>,
}

/// The plug preview: the cached scan, the mesh on show, and the work in flight
/// to replace it.
///
/// ⚠ One `Meshing` slot, not a queue. An edit made while a mesh is running is
/// answered by the *next* spawn, which reads the fields as they then stand — so
/// dragging a stepper converges on where it stopped instead of replaying every
/// value it passed through.
#[derive(Resource, Default)]
pub(crate) struct PlugView {
    cache: Cache,
    meshing: Option<Meshing>,
    /// The draft [`Self::mesh`] was built from — the whole re-mesh trigger.
    ///
    /// ⚠ Not `ShapeControls`'s change flag: the panel takes that resource
    /// mutably to *draw* the fields, so it reads as changed on every frame the
    /// screen is up.
    shown: Option<PlugDraft>,
    mesh: Option<IndexedMesh>,
    /// Bumped whenever [`Self::mesh`] is replaced. See [`Self::generation`].
    generation: u64,
}

impl PlugView {
    /// The mesh the viewport draws, once one has been built.
    pub(crate) const fn mesh(&self) -> Option<&IndexedMesh> {
        self.mesh.as_ref()
    }

    /// Which mesh [`Self::mesh`] is, counting from one.
    ///
    /// ⚠ The viewport gates its rebuild on this rather than on Bevy's change
    /// detection, because [`drive_plug_preview`] takes this resource mutably on
    /// every frame step 3 is up — so "changed" is true on frames where nothing
    /// was replaced, and the plug would be rebuilt from scratch at 60 Hz.
    pub(crate) const fn generation(&self) -> u64 {
        self.generation
    }

    /// True while the shape on screen is the generic proxy rather than this
    /// body — which the screen has to say, or a stand-in reads as the scan.
    pub(crate) const fn showing_proxy(&self) -> bool {
        self.mesh.is_some() && matches!(self.cache, Cache::Unavailable)
    }

    /// Drop everything built from the scan as it was.
    ///
    /// The mesh goes with the cache: it was cut out of that body, and leaving it
    /// up would preview a piece for a scan the user has since changed.
    fn invalidate(&mut self) {
        self.cache = Cache::Cold;
        self.meshing = None;
        self.shown = None;
        self.mesh = None;
    }

    /// Take delivery of a finished cache.
    fn land_cache(&mut self) {
        let Cache::Building(task) = &mut self.cache else {
            return;
        };
        let Some(built) = future::block_on(future::poll_once(task)) else {
            return;
        };
        self.cache = built.map_or(Cache::Unavailable, |p| Cache::Ready(Arc::new(p)));
    }

    /// Take delivery of a finished mesh.
    fn land_mesh(&mut self) {
        let Some(mut job) = self.meshing.take() else {
            return;
        };
        let Some(built) = future::block_on(future::poll_once(&mut job.task)) else {
            self.meshing = Some(job);
            return;
        };
        // ⚠ Recorded even when the mesh failed, or the driver re-spawns the
        // same failing draft on every frame the screen is up.
        self.shown = Some(job.draft);
        if let Some(mesh) = built {
            self.mesh = Some(mesh);
            self.generation += 1;
        }
    }

    /// Start building the cache, if it has not been tried.
    fn start_cache(&mut self, prep: Option<&PrepInput>) {
        if !matches!(self.cache, Cache::Cold) {
            return;
        }
        let Some(prep) = prep else {
            self.cache = Cache::Unavailable;
            return;
        };
        let (stl, toml) = (prep.cleaned_stl.clone(), prep.prep_toml.clone());
        self.cache = Cache::Building(AsyncComputeTaskPool::get().spawn(async move {
            // A scan that panics the SDF builder falls back to the proxy with
            // the note, rather than taking the app down with it.
            std::panic::catch_unwind(AssertUnwindSafe(|| PlugPreview::load(&stl, &toml).ok()))
                .ok()
                .flatten()
        }));
    }

    /// Start meshing `wanted`, if it is not what is already on screen.
    fn start_mesh(&mut self, wanted: &PlugDraft) {
        if self.meshing.is_some() || self.shown.as_ref() == Some(wanted) {
            return;
        }
        let scan = match &self.cache {
            Cache::Ready(cache) => Some(Arc::clone(cache)),
            Cache::Unavailable => None,
            // ⚠ Nothing is meshed while the cache builds. The scan stays on
            // screen for the half-second it takes, rather than flashing a
            // stand-in body the user would read as their own.
            Cache::Cold | Cache::Building(_) => return,
        };
        let draft = wanted.clone();
        let (ridges, inset) = (draft.ridges.clone(), draft.cavity_inset_m);
        let task = AsyncComputeTaskPool::get().spawn(async move {
            std::panic::catch_unwind(AssertUnwindSafe(|| match &scan {
                Some(cache) => cache.mesh(&ridges, inset),
                None => proxy_preview_mesh(&ridges),
            }))
            .ok()
        });
        self.meshing = Some(Meshing { draft, task });
    }
}

#[cfg(test)]
impl PlugView {
    /// Put `mesh` on show, as a landing job would.
    ///
    /// ⚠ For the viewport's own gates, which need a view in that state and have
    /// no task pool to reach it through. The driver's gates drive
    /// [`Self::land_mesh`] itself.
    pub(crate) fn show(&mut self, mesh: IndexedMesh) {
        self.mesh = Some(mesh);
        self.generation += 1;
    }
}

/// Keep the preview in step with the fields while step 3 is on screen.
pub(crate) fn drive_plug_preview(
    mut view: ResMut<PlugView>,
    studio: Res<Studio>,
    shape: Res<ShapeControls>,
    scan: Res<ScanEdit>,
) {
    // ⚠ Ahead of the step gate, because a step-2 edit lands while step 3 is off
    // screen: a cache built before it would preview a body the user has already
    // changed, and nothing on step 3 would ever notice.
    if scan.is_changed() {
        view.invalidate();
    }
    if studio.cursor.viewed() != Step::ShapePiece {
        return;
    }
    view.land_cache();
    view.land_mesh();
    view.start_cache(studio.project.prep());
    view.start_mesh(&shape.plug_draft());
}

#[cfg(test)]
pub(crate) mod tests {
    #![allow(clippy::expect_used)]

    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    use cf_studio_core::{PrepInput, Project, ScanInput};
    use cf_studio_gui::{StepBoxState, WizardCursor};
    use mesh_types::Bounded;

    use super::*;

    /// The 12 triangles of an axis-aligned box, by corner index — bit 0 is x,
    /// bit 1 is y, bit 2 is z.
    ///
    /// ⚠ Winding is not set, and does not need to be: the scan SDF takes its
    /// distance from an *unsigned* mesh distance and its sign from a flood fill,
    /// so a back-to-front triangle changes nothing it reads.
    const BOX_FACES: [[usize; 3]; 12] = [
        [0, 1, 3],
        [0, 3, 2],
        [4, 5, 7],
        [4, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [0, 2, 6],
        [0, 6, 4],
        [1, 3, 7],
        [1, 7, 5],
    ];

    /// A closed 40 mm box, in meters.
    ///
    /// ⚠ Small on purpose. The flood fill covers the scan's AABB at the
    /// preview's 2 mm cells, so a metre-wide fixture would ask for 125 million
    /// of them and this suite would never finish.
    fn box_stl() -> String {
        let corner = |i: usize| {
            [
                if i & 1 == 0 { -0.02 } else { 0.02 },
                if i & 2 == 0 { -0.02 } else { 0.02 },
                if i & 4 == 0 { 0.0 } else { 0.04 },
            ]
        };
        let mut stl = String::from("solid box\n");
        for face in BOX_FACES {
            stl.push_str("facet normal 0 0 0\n  outer loop\n");
            for i in face {
                let [x, y, z] = corner(i);
                stl.push_str(&format!("    vertex {x} {y} {z}\n"));
            }
            stl.push_str("  endloop\nendfacet\n");
        }
        stl.push_str("endsolid box\n");
        stl
    }

    /// That box and a centerline up its middle, on disk — the pair
    /// [`PlugPreview::load`] reads.
    pub(crate) fn a_cleaned_scan(tag: &str) -> PrepInput {
        let dir = fixture_dir(tag);
        std::fs::create_dir_all(&dir).expect("a temp dir to write the fixture into");
        let prep = PrepInput {
            cleaned_stl: dir.join("box.cleaned.stl"),
            prep_toml: dir.join("box.prep.toml"),
        };
        std::fs::write(&prep.cleaned_stl, box_stl()).expect("the fixture scan writes");
        std::fs::write(
            &prep.prep_toml,
            "[centerline]\npoints_m = [[0.0, 0.0, 0.004], [0.0, 0.0, 0.036]]\n",
        )
        .expect("the fixture prep writes");
        prep
    }

    pub(crate) fn fixture_dir(tag: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "cf-studio-gui-preview-{tag}-{}",
            std::process::id()
        ))
    }

    /// Paths that name a cleaned scan which is not there.
    pub(crate) fn a_missing_scan() -> PrepInput {
        let dir = fixture_dir("absent");
        PrepInput {
            cleaned_stl: dir.join("absent.cleaned.stl"),
            prep_toml: dir.join("absent.prep.toml"),
        }
    }

    /// A project cleaned as far as step 3, against `prep`.
    pub(crate) fn cleaned(prep: PrepInput) -> Project {
        let mut project = Project::new("preview");
        project.set_scan(ScanInput {
            source_path: PathBuf::from("box.stl"),
        });
        project
            .set_prep(prep)
            .expect("each artifact is set in workflow order");
        project
    }

    /// The driver, a task pool, and the resources it reads — no window, no
    /// renderer.
    fn app_on(step: Step, project: Project) -> App {
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<ScanEdit>()
            .init_resource::<ShapeControls>()
            .init_resource::<PlugView>()
            .insert_resource(Studio {
                project,
                cursor: WizardCursor::new(step),
                ..Studio::default()
            })
            .add_systems(Update, drive_plug_preview);
        app
    }

    /// Run frames until the preview answers for what the fields now say.
    ///
    /// ⚠ The condition is convergence, not "a mesh appeared". A driver that
    /// lands one mesh and then never re-spawns satisfies the latter, and that
    /// stall is the failure the single in-flight slot is one edit away from.
    pub(crate) fn settle(app: &mut App) {
        const DEADLINE: Duration = Duration::from_secs(20);
        let deadline = Instant::now() + DEADLINE;
        loop {
            app.update();
            let wanted = app.world().resource::<ShapeControls>().plug_draft();
            let view = app.world().resource::<PlugView>();
            if view.meshing.is_none() && view.shown.as_ref() == Some(&wanted) {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "the preview never caught up with the fields"
            );
        }
    }

    fn view(app: &App) -> &PlugView {
        app.world().resource::<PlugView>()
    }

    fn set_step(app: &mut App, step: Step) {
        app.world_mut().resource_mut::<Studio>().cursor = WizardCursor::new(step);
    }

    fn set_cavity(app: &mut App, mm: i32) {
        app.world_mut()
            .resource_mut::<ShapeControls>()
            .cavity_mm
            .state = StepBoxState::new(mm);
    }

    /// How big the piece on show is.
    fn size(app: &App) -> f64 {
        view(app)
            .mesh()
            .map(|m| m.aabb().diagonal())
            .expect("a piece is on show")
    }

    /// ★★ The whole point of the screen: the shape on show is the shape the
    /// fields describe, and a deeper cavity leaves a smaller piece.
    ///
    /// ⚠ The oracle is the plug's own size, not a second call to the mesher.
    /// Re-running `PlugPreview::mesh` on what the fields say would agree with
    /// itself even if the driver handed the mesher a number the fields never
    /// held; the inset is the one field whose consequence can be measured off
    /// the result instead.
    #[test]
    fn a_deeper_cavity_previews_a_smaller_piece() {
        let prep = a_cleaned_scan("inset");
        let mut app = app_on(Step::ShapePiece, cleaned(prep.clone()));

        set_cavity(&mut app, 2);
        settle(&mut app);
        let shallow = size(&app);

        set_cavity(&mut app, 8);
        settle(&mut app);
        let deep = size(&app);

        assert!(
            deep < shallow,
            "8 mm in from the surface must leave less of the body than 2 mm did: \
             {deep} vs {shallow}"
        );
        assert!(
            !view(&app).showing_proxy(),
            "a readable scan is previewed as itself, with no stand-in note"
        );
        let _ = std::fs::remove_dir_all(fixture_dir("inset"));
    }

    /// ★ The other half of that: an edit anywhere in the ridge editor must reach
    /// the shape too, not only the one field with a size to measure.
    ///
    /// Runs on the stand-in, which needs no scan on disk and no flood fill.
    #[test]
    fn switching_the_ridges_on_changes_the_shape_on_show() {
        let mut app = app_on(Step::ShapePiece, cleaned(a_missing_scan()));

        settle(&mut app);
        let smooth = view(&app).mesh().map(|m| m.vertices.len());

        app.world_mut()
            .resource_mut::<ShapeControls>()
            .ridges
            .enabled = true;
        settle(&mut app);
        let ridged = view(&app).mesh().map(|m| m.vertices.len());

        assert!(smooth.is_some() && ridged.is_some(), "both meshed");
        assert_ne!(
            smooth, ridged,
            "cutting ridges into the piece must change the piece: {smooth:?}"
        );
    }

    /// ⚠ A stand-in shown silently reads as the user's own body. The screen's
    /// note is driven off this, so this is where the honesty is decided.
    #[test]
    fn a_scan_that_cannot_be_read_previews_a_stand_in_and_says_so() {
        let mut app = app_on(Step::ShapePiece, cleaned(a_missing_scan()));

        settle(&mut app);

        assert!(
            view(&app).mesh().is_some(),
            "a shape still reaches the view"
        );
        assert!(
            view(&app).showing_proxy(),
            "and it must own up to being a stand-in"
        );
    }

    /// ⚠⚠ Nothing is drawn while the cache builds — the scan stays up for the
    /// half-second it takes rather than flashing a stand-in the user would read
    /// as their own body, then swapping it out from under them.
    ///
    /// One frame, and one is enough: `start_cache` runs *after* `land_cache` in
    /// the same call, so no cache can have landed by the end of the first.
    #[test]
    fn nothing_is_previewed_while_the_scan_cache_is_still_building() {
        let mut app = app_on(Step::ShapePiece, cleaned(a_cleaned_scan("building")));

        app.update();

        // ⚠ That no stand-in was STARTED, not merely that none has landed.
        // Nothing can have landed after one frame either way, so asserting the
        // mesh alone passes just as well with the guard deleted.
        assert!(
            view(&app).meshing.is_none(),
            "no stand-in is even started while the real body is on its way"
        );
        assert!(
            view(&app).mesh().is_none(),
            "so the viewport holds the scan until the real piece is ready"
        );
        assert!(
            matches!(view(&app).cache, Cache::Building(_)),
            "and the reason is that the cache is still being built"
        );
        let _ = std::fs::remove_dir_all(fixture_dir("building"));
    }

    /// ★★ The stale-body trap. A step-2 edit lands while step 3 is off screen,
    /// and the cache is built from the scan as it *was* — so without this the
    /// user pages forward onto a piece cut from a body they already changed,
    /// and nothing on the screen is wrong enough to notice.
    #[test]
    fn an_edit_to_the_scan_drops_the_preview_built_from_it() {
        let mut app = app_on(Step::ShapePiece, cleaned(a_missing_scan()));
        settle(&mut app);
        assert!(view(&app).mesh().is_some(), "something to lose");

        // ⚠ Paged away first, because that is where the edit really happens.
        // Invalidating from behind the step gate looks right and catches
        // nothing: on step 2 the gate has already returned.
        set_step(&mut app, Step::CleanScan);
        app.world_mut().resource_mut::<ScanEdit>().set_changed();
        app.update();

        assert!(
            view(&app).mesh().is_none(),
            "the piece goes with the body it was cut from"
        );

        set_step(&mut app, Step::ShapePiece);
        settle(&mut app);
        assert!(
            view(&app).mesh().is_some(),
            "and is rebuilt from the scan as it now stands"
        );
    }

    /// ⚠ A project carrying no cleaned scan at all still gets a preview, and
    /// still owns up to it. Leaving the cache `Cold` for want of a prep would
    /// show nothing and explain nothing — the step would simply have no picture
    /// on it — and would re-ask for one on every frame besides.
    #[test]
    fn a_project_with_no_cleaned_scan_still_previews_a_stand_in() {
        let mut app = app_on(Step::ShapePiece, Project::new("no prep"));

        settle(&mut app);

        assert!(
            view(&app).showing_proxy(),
            "with nothing to cut a piece from, the stand-in is what there is"
        );
    }

    /// ⚠ The flood fill is hundreds of milliseconds. Starting it on a step that
    /// shows no preview spends that on nothing — and on step 2 it would be spent
    /// on a scan the user is still editing.
    #[test]
    fn no_preview_is_built_on_a_step_that_is_not_showing_one() {
        let mut app = app_on(Step::CleanScan, cleaned(a_cleaned_scan("offstep")));

        for _ in 0..8 {
            app.update();
        }

        assert!(view(&app).mesh().is_none(), "nothing is meshed");
        assert!(
            matches!(view(&app).cache, Cache::Cold),
            "and nothing was even started"
        );
        let _ = std::fs::remove_dir_all(fixture_dir("offstep"));
    }
}

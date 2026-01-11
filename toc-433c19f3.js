// Populate the sidebar
//
// This is a script, and not included directly in the page, to control the total size of the book.
// The TOC contains an entry for each page, so if each page includes a copy of the TOC,
// the total size of the page becomes O(n**2).
class MDBookSidebarScrollbox extends HTMLElement {
    constructor() {
        super();
    }
    connectedCallback() {
        this.innerHTML = '<ol class="chapter"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="index.html"><strong aria-hidden="true">1.</strong> Introduction</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/index.html"><strong aria-hidden="true">2.</strong> Workspace</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/overview.html"><strong aria-hidden="true">2.1.</strong> Workspace Overview</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/shared_dep_themes.html"><strong aria-hidden="true">2.2.</strong> Shared Dependency Themes</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/feature_flags.html"><strong aria-hidden="true">2.3.</strong> Feature Flags</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/integration_contracts.html"><strong aria-hidden="true">2.4.</strong> Integration Contracts</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/canonical_flows.html"><strong aria-hidden="true">2.5.</strong> Canonical Flows</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/readers_guide.html"><strong aria-hidden="true">2.6.</strong> Reader’s Guide</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/index.html"><strong aria-hidden="true">3.</strong> Building Apps</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/01_story_setup.html"><strong aria-hidden="true">3.1.</strong> We set the mission</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/02_app_stack.html"><strong aria-hidden="true">3.2.</strong> We choose a small stack</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/03_sim_core.html"><strong aria-hidden="true">3.3.</strong> We add the ship body (sim_core)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/04_vision_runtime.html"><strong aria-hidden="true">3.4.</strong> We add eyes (vision_runtime + vision_core)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/05_recorder.html"><strong aria-hidden="true">3.5.</strong> We add a recorder (capture_utils + data_contracts)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/06_dataset.html"><strong aria-hidden="true">3.6.</strong> We turn captures into datasets (burn_dataset)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/07_training.html"><strong aria-hidden="true">3.7.</strong> We teach the ship (models + training)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/08_inference.html"><strong aria-hidden="true">3.8.</strong> We deploy the brain (inference)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/09_tools_ops.html"><strong aria-hidden="true">3.9.</strong> We add mission control (cortenforge-tools + cli_support)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="20_building_apps/10_epilogue.html"><strong aria-hidden="true">3.10.</strong> We celebrate and choose the next adventure</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/index.html"><strong aria-hidden="true">4.</strong> Crate Overview</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/index.html"><strong aria-hidden="true">4.1.</strong> sim_core</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/index.html"><strong aria-hidden="true">4.2.</strong> vision_core</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/index.html"><strong aria-hidden="true">4.3.</strong> vision_runtime</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/index.html"><strong aria-hidden="true">4.4.</strong> data_contracts</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/index.html"><strong aria-hidden="true">4.5.</strong> capture_utils</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/index.html"><strong aria-hidden="true">4.6.</strong> models</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/index.html"><strong aria-hidden="true">4.7.</strong> training</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/index.html"><strong aria-hidden="true">4.8.</strong> inference</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/index.html"><strong aria-hidden="true">4.9.</strong> cli_support</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/index.html"><strong aria-hidden="true">4.10.</strong> burn_dataset</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/index.html"><strong aria-hidden="true">4.11.</strong> cortenforge-tools (shared)</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/index.html"><strong aria-hidden="true">4.12.</strong> cortenforge (umbrella)</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/deep_dives.html"><strong aria-hidden="true">5.</strong> Advanced: Crate Deep Dives</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/01_overview.html"><strong aria-hidden="true">5.1.</strong> sim_core</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/02_public_api.html"><strong aria-hidden="true">5.1.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/05_traits_and_generics.html"><strong aria-hidden="true">5.1.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/04_module_map.html"><strong aria-hidden="true">5.1.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/03_lifecycle.html"><strong aria-hidden="true">5.1.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/06_error_model.html"><strong aria-hidden="true">5.1.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.1.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/08_performance_notes.html"><strong aria-hidden="true">5.1.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/09_examples.html"><strong aria-hidden="true">5.1.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/sim_core/10_design_review.html"><strong aria-hidden="true">5.1.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/01_overview.html"><strong aria-hidden="true">5.2.</strong> vision_core</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/02_public_api.html"><strong aria-hidden="true">5.2.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/05_traits_and_generics.html"><strong aria-hidden="true">5.2.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/04_module_map.html"><strong aria-hidden="true">5.2.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/03_lifecycle.html"><strong aria-hidden="true">5.2.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/06_error_model.html"><strong aria-hidden="true">5.2.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.2.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/08_performance_notes.html"><strong aria-hidden="true">5.2.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/09_examples.html"><strong aria-hidden="true">5.2.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_core/10_design_review.html"><strong aria-hidden="true">5.2.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/01_overview.html"><strong aria-hidden="true">5.3.</strong> vision_runtime</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/02_public_api.html"><strong aria-hidden="true">5.3.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/05_traits_and_generics.html"><strong aria-hidden="true">5.3.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/04_module_map.html"><strong aria-hidden="true">5.3.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/03_lifecycle.html"><strong aria-hidden="true">5.3.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/06_error_model.html"><strong aria-hidden="true">5.3.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.3.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/08_performance_notes.html"><strong aria-hidden="true">5.3.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/09_examples.html"><strong aria-hidden="true">5.3.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/vision_runtime/10_design_review.html"><strong aria-hidden="true">5.3.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/01_overview.html"><strong aria-hidden="true">5.4.</strong> data_contracts</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/02_public_api.html"><strong aria-hidden="true">5.4.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/05_traits_and_generics.html"><strong aria-hidden="true">5.4.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/04_module_map.html"><strong aria-hidden="true">5.4.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/03_lifecycle.html"><strong aria-hidden="true">5.4.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/06_error_model.html"><strong aria-hidden="true">5.4.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.4.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/08_performance_notes.html"><strong aria-hidden="true">5.4.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/09_examples.html"><strong aria-hidden="true">5.4.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/data_contracts/10_design_review.html"><strong aria-hidden="true">5.4.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/01_overview.html"><strong aria-hidden="true">5.5.</strong> capture_utils</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/02_public_api.html"><strong aria-hidden="true">5.5.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/05_traits_and_generics.html"><strong aria-hidden="true">5.5.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/04_module_map.html"><strong aria-hidden="true">5.5.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/03_lifecycle.html"><strong aria-hidden="true">5.5.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/06_error_model.html"><strong aria-hidden="true">5.5.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.5.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/08_performance_notes.html"><strong aria-hidden="true">5.5.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/09_examples.html"><strong aria-hidden="true">5.5.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/capture_utils/10_design_review.html"><strong aria-hidden="true">5.5.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/01_overview.html"><strong aria-hidden="true">5.6.</strong> models</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/02_public_api.html"><strong aria-hidden="true">5.6.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/05_traits_and_generics.html"><strong aria-hidden="true">5.6.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/04_module_map.html"><strong aria-hidden="true">5.6.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/03_lifecycle.html"><strong aria-hidden="true">5.6.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/06_error_model.html"><strong aria-hidden="true">5.6.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.6.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/08_performance_notes.html"><strong aria-hidden="true">5.6.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/09_examples.html"><strong aria-hidden="true">5.6.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/models/10_design_review.html"><strong aria-hidden="true">5.6.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/01_overview.html"><strong aria-hidden="true">5.7.</strong> training</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/02_public_api.html"><strong aria-hidden="true">5.7.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/05_traits_and_generics.html"><strong aria-hidden="true">5.7.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/04_module_map.html"><strong aria-hidden="true">5.7.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/03_lifecycle.html"><strong aria-hidden="true">5.7.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/06_error_model.html"><strong aria-hidden="true">5.7.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.7.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/08_performance_notes.html"><strong aria-hidden="true">5.7.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/09_examples.html"><strong aria-hidden="true">5.7.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/training/10_design_review.html"><strong aria-hidden="true">5.7.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/01_overview.html"><strong aria-hidden="true">5.8.</strong> inference</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/02_public_api.html"><strong aria-hidden="true">5.8.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/05_traits_and_generics.html"><strong aria-hidden="true">5.8.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/04_module_map.html"><strong aria-hidden="true">5.8.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/03_lifecycle.html"><strong aria-hidden="true">5.8.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/06_error_model.html"><strong aria-hidden="true">5.8.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.8.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/08_performance_notes.html"><strong aria-hidden="true">5.8.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/09_examples.html"><strong aria-hidden="true">5.8.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/inference/10_design_review.html"><strong aria-hidden="true">5.8.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/01_overview.html"><strong aria-hidden="true">5.9.</strong> cli_support</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/02_public_api.html"><strong aria-hidden="true">5.9.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/05_traits_and_generics.html"><strong aria-hidden="true">5.9.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/04_module_map.html"><strong aria-hidden="true">5.9.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/03_lifecycle.html"><strong aria-hidden="true">5.9.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/06_error_model.html"><strong aria-hidden="true">5.9.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.9.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/08_performance_notes.html"><strong aria-hidden="true">5.9.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/09_examples.html"><strong aria-hidden="true">5.9.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cli_support/10_design_review.html"><strong aria-hidden="true">5.9.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/01_overview.html"><strong aria-hidden="true">5.10.</strong> burn_dataset</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/02_public_api.html"><strong aria-hidden="true">5.10.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/05_traits_and_generics.html"><strong aria-hidden="true">5.10.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/04_module_map.html"><strong aria-hidden="true">5.10.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/03_lifecycle.html"><strong aria-hidden="true">5.10.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/06_error_model.html"><strong aria-hidden="true">5.10.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.10.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/08_performance_notes.html"><strong aria-hidden="true">5.10.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/09_examples.html"><strong aria-hidden="true">5.10.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/burn_dataset/10_design_review.html"><strong aria-hidden="true">5.10.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/01_overview.html"><strong aria-hidden="true">5.11.</strong> cortenforge-tools (shared)</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/02_public_api.html"><strong aria-hidden="true">5.11.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/05_traits_and_generics.html"><strong aria-hidden="true">5.11.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/04_module_map.html"><strong aria-hidden="true">5.11.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/03_lifecycle.html"><strong aria-hidden="true">5.11.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/06_error_model.html"><strong aria-hidden="true">5.11.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.11.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/08_performance_notes.html"><strong aria-hidden="true">5.11.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/09_examples.html"><strong aria-hidden="true">5.11.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge_tools/10_design_review.html"><strong aria-hidden="true">5.11.9.</strong> Design Review</a></span></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/01_overview.html"><strong aria-hidden="true">5.12.</strong> cortenforge (umbrella)</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/02_public_api.html"><strong aria-hidden="true">5.12.1.</strong> Public API</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/05_traits_and_generics.html"><strong aria-hidden="true">5.12.2.</strong> Traits &amp; Generics</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/04_module_map.html"><strong aria-hidden="true">5.12.3.</strong> Modules</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/03_lifecycle.html"><strong aria-hidden="true">5.12.4.</strong> Data Flow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/06_error_model.html"><strong aria-hidden="true">5.12.5.</strong> Error Model</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/07_ownership_and_concurrency.html"><strong aria-hidden="true">5.12.6.</strong> Ownership &amp; Concurrency</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/08_performance_notes.html"><strong aria-hidden="true">5.12.7.</strong> Performance</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/09_examples.html"><strong aria-hidden="true">5.12.8.</strong> Examples</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="10_crates/cortenforge/10_design_review.html"><strong aria-hidden="true">5.12.9.</strong> Design Review</a></span></li></ol></li></ol><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="30_reference/index.html"><strong aria-hidden="true">6.</strong> Reference</a></span><ol class="section"><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/workspace_metadata.html"><strong aria-hidden="true">6.1.</strong> Workspace Metadata</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/build_and_run.html"><strong aria-hidden="true">6.2.</strong> Build &amp; Dev Workflow</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/dev_loop.html"><strong aria-hidden="true">6.3.</strong> Dev Loop</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/reproducibility.html"><strong aria-hidden="true">6.4.</strong> Reproducibility</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/glossary.html"><strong aria-hidden="true">6.5.</strong> Glossary</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/style_guide.html"><strong aria-hidden="true">6.6.</strong> Style Guide</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/linking_style.html"><strong aria-hidden="true">6.7.</strong> Linking Style</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/docsrs_alignment.html"><strong aria-hidden="true">6.8.</strong> Docs.rs Alignment</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/maintenance_routine.html"><strong aria-hidden="true">6.9.</strong> Maintenance Routine</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/quality_gates.html"><strong aria-hidden="true">6.10.</strong> Quality Gates</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/open_questions.html"><strong aria-hidden="true">6.11.</strong> Open Questions</a></span></li><li class="chapter-item expanded "><span class="chapter-link-wrapper"><a href="00_workspace/changelog.html"><strong aria-hidden="true">6.12.</strong> Changelog</a></span></li></ol></li></ol>';
        // Set the current, active page, and reveal it if it's hidden
        let current_page = document.location.href.toString().split('#')[0].split('?')[0];
        if (current_page.endsWith('/')) {
            current_page += 'index.html';
        }
        const links = Array.prototype.slice.call(this.querySelectorAll('a'));
        const l = links.length;
        for (let i = 0; i < l; ++i) {
            const link = links[i];
            const href = link.getAttribute('href');
            if (href && !href.startsWith('#') && !/^(?:[a-z+]+:)?\/\//.test(href)) {
                link.href = path_to_root + href;
            }
            // The 'index' page is supposed to alias the first chapter in the book.
            if (link.href === current_page
                || i === 0
                && path_to_root === ''
                && current_page.endsWith('/index.html')) {
                link.classList.add('active');
                let parent = link.parentElement;
                while (parent) {
                    if (parent.tagName === 'LI' && parent.classList.contains('chapter-item')) {
                        parent.classList.add('expanded');
                    }
                    parent = parent.parentElement;
                }
            }
        }
        // Track and set sidebar scroll position
        this.addEventListener('click', e => {
            if (e.target.tagName === 'A') {
                const clientRect = e.target.getBoundingClientRect();
                const sidebarRect = this.getBoundingClientRect();
                sessionStorage.setItem('sidebar-scroll-offset', clientRect.top - sidebarRect.top);
            }
        }, { passive: true });
        const sidebarScrollOffset = sessionStorage.getItem('sidebar-scroll-offset');
        sessionStorage.removeItem('sidebar-scroll-offset');
        if (sidebarScrollOffset !== null) {
            // preserve sidebar scroll position when navigating via links within sidebar
            const activeSection = this.querySelector('.active');
            if (activeSection) {
                const clientRect = activeSection.getBoundingClientRect();
                const sidebarRect = this.getBoundingClientRect();
                const currentOffset = clientRect.top - sidebarRect.top;
                this.scrollTop += currentOffset - parseFloat(sidebarScrollOffset);
            }
        } else {
            // scroll sidebar to current active section when navigating via
            // 'next/previous chapter' buttons
            const activeSection = document.querySelector('#mdbook-sidebar .active');
            if (activeSection) {
                activeSection.scrollIntoView({ block: 'center' });
            }
        }
        // Toggle buttons
        const sidebarAnchorToggles = document.querySelectorAll('.chapter-fold-toggle');
        function toggleSection(ev) {
            ev.currentTarget.parentElement.parentElement.classList.toggle('expanded');
        }
        Array.from(sidebarAnchorToggles).forEach(el => {
            el.addEventListener('click', toggleSection);
        });
    }
}
window.customElements.define('mdbook-sidebar-scrollbox', MDBookSidebarScrollbox);


// ---------------------------------------------------------------------------
// Support for dynamically adding headers to the sidebar.

(function() {
    // This is used to detect which direction the page has scrolled since the
    // last scroll event.
    let lastKnownScrollPosition = 0;
    // This is the threshold in px from the top of the screen where it will
    // consider a header the "current" header when scrolling down.
    const defaultDownThreshold = 150;
    // Same as defaultDownThreshold, except when scrolling up.
    const defaultUpThreshold = 300;
    // The threshold is a virtual horizontal line on the screen where it
    // considers the "current" header to be above the line. The threshold is
    // modified dynamically to handle headers that are near the bottom of the
    // screen, and to slightly offset the behavior when scrolling up vs down.
    let threshold = defaultDownThreshold;
    // This is used to disable updates while scrolling. This is needed when
    // clicking the header in the sidebar, which triggers a scroll event. It
    // is somewhat finicky to detect when the scroll has finished, so this
    // uses a relatively dumb system of disabling scroll updates for a short
    // time after the click.
    let disableScroll = false;
    // Array of header elements on the page.
    let headers;
    // Array of li elements that are initially collapsed headers in the sidebar.
    // I'm not sure why eslint seems to have a false positive here.
    // eslint-disable-next-line prefer-const
    let headerToggles = [];
    // This is a debugging tool for the threshold which you can enable in the console.
    let thresholdDebug = false;

    // Updates the threshold based on the scroll position.
    function updateThreshold() {
        const scrollTop = window.pageYOffset || document.documentElement.scrollTop;
        const windowHeight = window.innerHeight;
        const documentHeight = document.documentElement.scrollHeight;

        // The number of pixels below the viewport, at most documentHeight.
        // This is used to push the threshold down to the bottom of the page
        // as the user scrolls towards the bottom.
        const pixelsBelow = Math.max(0, documentHeight - (scrollTop + windowHeight));
        // The number of pixels above the viewport, at least defaultDownThreshold.
        // Similar to pixelsBelow, this is used to push the threshold back towards
        // the top when reaching the top of the page.
        const pixelsAbove = Math.max(0, defaultDownThreshold - scrollTop);
        // How much the threshold should be offset once it gets close to the
        // bottom of the page.
        const bottomAdd = Math.max(0, windowHeight - pixelsBelow - defaultDownThreshold);
        let adjustedBottomAdd = bottomAdd;

        // Adjusts bottomAdd for a small document. The calculation above
        // assumes the document is at least twice the windowheight in size. If
        // it is less than that, then bottomAdd needs to be shrunk
        // proportional to the difference in size.
        if (documentHeight < windowHeight * 2) {
            const maxPixelsBelow = documentHeight - windowHeight;
            const t = 1 - pixelsBelow / Math.max(1, maxPixelsBelow);
            const clamp = Math.max(0, Math.min(1, t));
            adjustedBottomAdd *= clamp;
        }

        let scrollingDown = true;
        if (scrollTop < lastKnownScrollPosition) {
            scrollingDown = false;
        }

        if (scrollingDown) {
            // When scrolling down, move the threshold up towards the default
            // downwards threshold position. If near the bottom of the page,
            // adjustedBottomAdd will offset the threshold towards the bottom
            // of the page.
            const amountScrolledDown = scrollTop - lastKnownScrollPosition;
            const adjustedDefault = defaultDownThreshold + adjustedBottomAdd;
            threshold = Math.max(adjustedDefault, threshold - amountScrolledDown);
        } else {
            // When scrolling up, move the threshold down towards the default
            // upwards threshold position. If near the bottom of the page,
            // quickly transition the threshold back up where it normally
            // belongs.
            const amountScrolledUp = lastKnownScrollPosition - scrollTop;
            const adjustedDefault = defaultUpThreshold - pixelsAbove
                + Math.max(0, adjustedBottomAdd - defaultDownThreshold);
            threshold = Math.min(adjustedDefault, threshold + amountScrolledUp);
        }

        if (documentHeight <= windowHeight) {
            threshold = 0;
        }

        if (thresholdDebug) {
            const id = 'mdbook-threshold-debug-data';
            let data = document.getElementById(id);
            if (data === null) {
                data = document.createElement('div');
                data.id = id;
                data.style.cssText = `
                    position: fixed;
                    top: 50px;
                    right: 10px;
                    background-color: 0xeeeeee;
                    z-index: 9999;
                    pointer-events: none;
                `;
                document.body.appendChild(data);
            }
            data.innerHTML = `
                <table>
                  <tr><td>documentHeight</td><td>${documentHeight.toFixed(1)}</td></tr>
                  <tr><td>windowHeight</td><td>${windowHeight.toFixed(1)}</td></tr>
                  <tr><td>scrollTop</td><td>${scrollTop.toFixed(1)}</td></tr>
                  <tr><td>pixelsAbove</td><td>${pixelsAbove.toFixed(1)}</td></tr>
                  <tr><td>pixelsBelow</td><td>${pixelsBelow.toFixed(1)}</td></tr>
                  <tr><td>bottomAdd</td><td>${bottomAdd.toFixed(1)}</td></tr>
                  <tr><td>adjustedBottomAdd</td><td>${adjustedBottomAdd.toFixed(1)}</td></tr>
                  <tr><td>scrollingDown</td><td>${scrollingDown}</td></tr>
                  <tr><td>threshold</td><td>${threshold.toFixed(1)}</td></tr>
                </table>
            `;
            drawDebugLine();
        }

        lastKnownScrollPosition = scrollTop;
    }

    function drawDebugLine() {
        if (!document.body) {
            return;
        }
        const id = 'mdbook-threshold-debug-line';
        const existingLine = document.getElementById(id);
        if (existingLine) {
            existingLine.remove();
        }
        const line = document.createElement('div');
        line.id = id;
        line.style.cssText = `
            position: fixed;
            top: ${threshold}px;
            left: 0;
            width: 100vw;
            height: 2px;
            background-color: red;
            z-index: 9999;
            pointer-events: none;
        `;
        document.body.appendChild(line);
    }

    function mdbookEnableThresholdDebug() {
        thresholdDebug = true;
        updateThreshold();
        drawDebugLine();
    }

    window.mdbookEnableThresholdDebug = mdbookEnableThresholdDebug;

    // Updates which headers in the sidebar should be expanded. If the current
    // header is inside a collapsed group, then it, and all its parents should
    // be expanded.
    function updateHeaderExpanded(currentA) {
        // Add expanded to all header-item li ancestors.
        let current = currentA.parentElement;
        while (current) {
            if (current.tagName === 'LI' && current.classList.contains('header-item')) {
                current.classList.add('expanded');
            }
            current = current.parentElement;
        }
    }

    // Updates which header is marked as the "current" header in the sidebar.
    // This is done with a virtual Y threshold, where headers at or below
    // that line will be considered the current one.
    function updateCurrentHeader() {
        if (!headers || !headers.length) {
            return;
        }

        // Reset the classes, which will be rebuilt below.
        const els = document.getElementsByClassName('current-header');
        for (const el of els) {
            el.classList.remove('current-header');
        }
        for (const toggle of headerToggles) {
            toggle.classList.remove('expanded');
        }

        // Find the last header that is above the threshold.
        let lastHeader = null;
        for (const header of headers) {
            const rect = header.getBoundingClientRect();
            if (rect.top <= threshold) {
                lastHeader = header;
            } else {
                break;
            }
        }
        if (lastHeader === null) {
            lastHeader = headers[0];
            const rect = lastHeader.getBoundingClientRect();
            const windowHeight = window.innerHeight;
            if (rect.top >= windowHeight) {
                return;
            }
        }

        // Get the anchor in the summary.
        const href = '#' + lastHeader.id;
        const a = [...document.querySelectorAll('.header-in-summary')]
            .find(element => element.getAttribute('href') === href);
        if (!a) {
            return;
        }

        a.classList.add('current-header');

        updateHeaderExpanded(a);
    }

    // Updates which header is "current" based on the threshold line.
    function reloadCurrentHeader() {
        if (disableScroll) {
            return;
        }
        updateThreshold();
        updateCurrentHeader();
    }


    // When clicking on a header in the sidebar, this adjusts the threshold so
    // that it is located next to the header. This is so that header becomes
    // "current".
    function headerThresholdClick(event) {
        // See disableScroll description why this is done.
        disableScroll = true;
        setTimeout(() => {
            disableScroll = false;
        }, 100);
        // requestAnimationFrame is used to delay the update of the "current"
        // header until after the scroll is done, and the header is in the new
        // position.
        requestAnimationFrame(() => {
            requestAnimationFrame(() => {
                // Closest is needed because if it has child elements like <code>.
                const a = event.target.closest('a');
                const href = a.getAttribute('href');
                const targetId = href.substring(1);
                const targetElement = document.getElementById(targetId);
                if (targetElement) {
                    threshold = targetElement.getBoundingClientRect().bottom;
                    updateCurrentHeader();
                }
            });
        });
    }

    // Takes the nodes from the given head and copies them over to the
    // destination, along with some filtering.
    function filterHeader(source, dest) {
        const clone = source.cloneNode(true);
        clone.querySelectorAll('mark').forEach(mark => {
            mark.replaceWith(...mark.childNodes);
        });
        dest.append(...clone.childNodes);
    }

    // Scans page for headers and adds them to the sidebar.
    document.addEventListener('DOMContentLoaded', function() {
        const activeSection = document.querySelector('#mdbook-sidebar .active');
        if (activeSection === null) {
            return;
        }

        const main = document.getElementsByTagName('main')[0];
        headers = Array.from(main.querySelectorAll('h2, h3, h4, h5, h6'))
            .filter(h => h.id !== '' && h.children.length && h.children[0].tagName === 'A');

        if (headers.length === 0) {
            return;
        }

        // Build a tree of headers in the sidebar.

        const stack = [];

        const firstLevel = parseInt(headers[0].tagName.charAt(1));
        for (let i = 1; i < firstLevel; i++) {
            const ol = document.createElement('ol');
            ol.classList.add('section');
            if (stack.length > 0) {
                stack[stack.length - 1].ol.appendChild(ol);
            }
            stack.push({level: i + 1, ol: ol});
        }

        // The level where it will start folding deeply nested headers.
        const foldLevel = 3;

        for (let i = 0; i < headers.length; i++) {
            const header = headers[i];
            const level = parseInt(header.tagName.charAt(1));

            const currentLevel = stack[stack.length - 1].level;
            if (level > currentLevel) {
                // Begin nesting to this level.
                for (let nextLevel = currentLevel + 1; nextLevel <= level; nextLevel++) {
                    const ol = document.createElement('ol');
                    ol.classList.add('section');
                    const last = stack[stack.length - 1];
                    const lastChild = last.ol.lastChild;
                    // Handle the case where jumping more than one nesting
                    // level, which doesn't have a list item to place this new
                    // list inside of.
                    if (lastChild) {
                        lastChild.appendChild(ol);
                    } else {
                        last.ol.appendChild(ol);
                    }
                    stack.push({level: nextLevel, ol: ol});
                }
            } else if (level < currentLevel) {
                while (stack.length > 1 && stack[stack.length - 1].level > level) {
                    stack.pop();
                }
            }

            const li = document.createElement('li');
            li.classList.add('header-item');
            li.classList.add('expanded');
            if (level < foldLevel) {
                li.classList.add('expanded');
            }
            const span = document.createElement('span');
            span.classList.add('chapter-link-wrapper');
            const a = document.createElement('a');
            span.appendChild(a);
            a.href = '#' + header.id;
            a.classList.add('header-in-summary');
            filterHeader(header.children[0], a);
            a.addEventListener('click', headerThresholdClick);
            const nextHeader = headers[i + 1];
            if (nextHeader !== undefined) {
                const nextLevel = parseInt(nextHeader.tagName.charAt(1));
                if (nextLevel > level && level >= foldLevel) {
                    const toggle = document.createElement('a');
                    toggle.classList.add('chapter-fold-toggle');
                    toggle.classList.add('header-toggle');
                    toggle.addEventListener('click', () => {
                        li.classList.toggle('expanded');
                    });
                    const toggleDiv = document.createElement('div');
                    toggleDiv.textContent = '❱';
                    toggle.appendChild(toggleDiv);
                    span.appendChild(toggle);
                    headerToggles.push(li);
                }
            }
            li.appendChild(span);

            const currentParent = stack[stack.length - 1];
            currentParent.ol.appendChild(li);
        }

        const onThisPage = document.createElement('div');
        onThisPage.classList.add('on-this-page');
        onThisPage.append(stack[0].ol);
        const activeItemSpan = activeSection.parentElement;
        activeItemSpan.after(onThisPage);
    });

    document.addEventListener('DOMContentLoaded', reloadCurrentHeader);
    document.addEventListener('scroll', reloadCurrentHeader, { passive: true });
})();


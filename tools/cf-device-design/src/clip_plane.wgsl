// cf-device-design fit-viz rung 1 — centerline-anchored clip-plane
// fragment shader. Discards fragments on the kept-OUT side of the
// plane before the standard PBR forward path runs; everything else
// is the stock bevy_pbr forward fragment (mirrors
// `bevy_pbr/src/render/pbr.wgsl`, minus the prepass/deferred,
// meshlet, OIT, visibility-dither, and forward-decal branches —
// none of which cf-device-design uses).
//
// The plane is in RENDER-FRAME meters (the same UpAxis::PlusZ +
// RenderScale space the rendered meshes live in). Conversion from
// physics frame → render frame happens at the uniform-push boundary
// (sub-leaf 4, Rust side), so this shader does a single dot-product
// + sign test per fragment.

#import bevy_pbr::{
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing},
    pbr_types::STANDARD_MATERIAL_FLAGS_UNLIT_BIT,
    forward_io::{VertexOutput, FragmentOutput},
}

struct ClipPlane {
    // .xyz = unit normal (render-frame), .w = dot(origin, normal).
    // Plane equation: dot(world_pos, normal) - w = 0.
    // Kept half is dot(world_pos, normal) - w >= 0.
    plane: vec4<f32>,
    // Non-zero = clipping enabled. Zero = pass-through (no discard
    // test); the kept half is the entire world.
    enabled: u32,
};

// `#{MATERIAL_BIND_GROUP}` is a Bevy shader def that expands to the
// current material bind group index (3 in Bevy 0.18 — changed from
// 2 in 0.17). Using the def keeps this future-proof against further
// bind-group renumbering.
@group(#{MATERIAL_BIND_GROUP}) @binding(100) var<uniform> clip: ClipPlane;

@fragment
fn fragment(
    in: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> FragmentOutput {
    if (clip.enabled != 0u) {
        let signed_distance = dot(in.world_position.xyz, clip.plane.xyz) - clip.plane.w;
        if (signed_distance < 0.0) {
            discard;
        }
    }

    var pbr_input = pbr_input_from_standard_material(in, is_front);
    pbr_input.material.base_color = alpha_discard(
        pbr_input.material,
        pbr_input.material.base_color,
    );

    var out: FragmentOutput;
    if ((pbr_input.material.flags & STANDARD_MATERIAL_FLAGS_UNLIT_BIT) == 0u) {
        out.color = apply_pbr_lighting(pbr_input);
    } else {
        out.color = pbr_input.material.base_color;
    }
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}

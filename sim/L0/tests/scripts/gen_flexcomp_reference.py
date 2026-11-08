"""MuJoCo 3.5.0's values for `sim/L0/mjcf/tests/flexcomp_body.rs`: each case's document loaded on the
unfused oracle, one forward at qpos0, and the values the tests compare printed as Rust constants.

Run with the oracle's Python (`sim/L0/tests/scripts/build_mujoco_oracle.sh`):
    <oracle>/venv/bin/python -I gen_flexcomp_reference.py > out.rs
The tests embed the output after their GENERATED marker, as rustfmt lays it out; every value is printed
with `repr`, which round-trips an f64 exactly.
"""
import sys

import mujoco

CASES = {
    # grids (L152, L52 U08/U09)
    "GRID1": '<flexcomp name="f" type="grid" count="3 1 1" spacing="0.1 0.1 0.1" dim="1"/>',
    "GRID2": '<flexcomp name="f" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/>',
    "GRID2_3X2": '<flexcomp name="f" type="grid" count="3 2 1" spacing="0.1 0.2 0.1" dim="2"/>',
    "GRID3": '<flexcomp name="f" type="grid" count="2 2 2" spacing="0.1 0.1 0.1" dim="3"/>',
    "CIRCLE": '<flexcomp name="f" type="circle" count="5 1 1" spacing="0.1 0.1 0.1" dim="1"/>',
    # the other makers
    "BOX3": '<flexcomp name="f" type="box" count="3 3 3" spacing="0.1 0.1 0.1" dim="3"/>',
    "BOX2": '<flexcomp name="f" type="box" count="2 2 2" spacing="0.1 0.1 0.1" dim="2"/>',
    "CYLINDER2": '<flexcomp name="f" type="cylinder" count="3 3 3" spacing="0.1 0.1 0.1" dim="2"/>',
    "ELLIPSOID3": '<flexcomp name="f" type="ellipsoid" count="3 3 3" spacing="0.1 0.2 0.3" dim="3"/>',
    "SQUARE": '<flexcomp name="f" type="square" count="3 3 1" spacing="0.1 0.1 0.1" dim="3"/>',
    "DISC": '<flexcomp name="f" type="disc" count="3 3 1" spacing="0.1 0.1 0.1"/>',
    "DIRECT": '<flexcomp name="f" type="direct" dim="1" point="0 0 0  1 0 0  2 0 0" element="0 2" scale="2 1 1"/>',
    "GRID_WITH_POINTS": '<flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1" point="5 5 5"/>',
    # the pose (L52 U08 'pos', L312)
    "POSE": '<flexcomp name="f" type="grid" count="3 1 1" spacing="0.1 0.1 0.1" dim="1" pos="0 0 1" euler="0 0 90"/>',
    "QUAT_RAW": '<flexcomp name="f" type="grid" count="3 1 1" spacing="0.1 0.1 0.1" dim="1" quat="0 0 0 2"/>',
    "QUAT_ZERO": '<flexcomp name="f" type="grid" count="3 1 1" spacing="0.1 0.1 0.1" dim="1" quat="0 0 0 0"/>',
    # pins and rigid (L395)
    "PINS": ('<flexcomp name="f" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" mass="9">'
             '<pin id="0"/><pin range="4 5"/><pin grid="2 2"/></flexcomp>'),
    "PIN_GRIDRANGE": ('<flexcomp name="f" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2">'
                      '<pin gridrange="0 0 0 2"/></flexcomp>'),
    "RIGID": '<flexcomp name="f" type="grid" count="3 1 1" spacing="0.1 0.1 0.1" dim="1" rigid="true"/>',
    "ALL_PINNED_EQ": ('<flexcomp name="f" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2">'
                      '<pin range="0 3"/><edge equality="true"/></flexcomp>'),
    "RIGID_EQ": ('<flexcomp name="f" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2" rigid="true">'
                 '<edge equality="true"/></flexcomp>'),
    "STATIC_HOLDER": ('<body name="b" pos="1 2 3"><flexcomp name="f" type="grid" count="3 1 1" '
                      'spacing="0.1 0.1 0.1" dim="1"><pin id="0"/></flexcomp></body>'),
    # spheres and the class (L325's first half, the defaults)
    "SPHERES": ('<flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1" radius="0.01">'
                '<contact vertcollide="true"/></flexcomp>'),
}
DEFAULTS = ('<default><joint damping="0.5" range="-1 1"/><geom contype="3" group="1" friction="0.7"/>'
            '<default class="X"><joint damping="0.7"/></default><default class="Y"><joint damping="0.9"/>'
            '</default></default>')
CLASS_CASES = {
    "CLASS_MAIN": '<flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"><contact vertcollide="true"/></flexcomp>',
    "CLASS_HOLDER": '<body name="b" childclass="Y"><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></body>',
    "CLASS_FRAME_IGNORED": ('<body name="b" childclass="Y"><frame childclass="X" pos="1 2 3"><flexcomp name="f" '
                            'type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></frame></body>'),
    "CLASS_BODY_IN_FRAME": ('<frame childclass="X" pos="1 0 0"><body name="b"><flexcomp name="f" type="grid" '
                            'count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></body></frame>'),
}
ORDER_CASES = {
    "ORDER_FRAME": ('<body name="a"/><frame pos="1 0 0"><body name="c"/><flexcomp name="f" type="grid" '
                    'count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/><body name="d"/></frame><body name="z"/>'),
}
# whole documents (sections outside the world body)
DOCS = {
    "NESTED_ORDER": ('<mujoco><deformable><flex name="d" dim="1" body="A B" element="0 1"/>'
                     '</deformable><worldbody><body name="A"><flexcomp name="f1" type="grid" count="2 1 1" '
                     'spacing="0.1 0.1 0.1" dim="1"/><body name="B" pos="0 0 1"><flexcomp name="f2" type="grid" '
                     'count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></body><flexcomp name="f1b" type="grid" '
                     'count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></body><flexcomp name="f3" type="grid" '
                     'count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></worldbody></mujoco>'),
    "INERTIAFROMGEOM": ('<mujoco><compiler inertiafromgeom="true"/><worldbody><flexcomp name="f" type="grid" '
                        'count="2 1 1" spacing="0.1 0.1 0.1" dim="1" radius="0.01"><contact vertcollide="true"/>'
                        '</flexcomp></worldbody></mujoco>'),
    "SETTOTALMASS": ('<mujoco><compiler settotalmass="10"/><worldbody><flexcomp name="f" type="grid" '
                     'count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></worldbody></mujoco>'),
    "BOUNDMASS": ('<mujoco><compiler boundmass="0.5" boundinertia="0.001"/><worldbody><flexcomp name="f" '
                  'type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></worldbody></mujoco>'),
    "RIGID_BOUNDMASS_FREE": ('<mujoco><compiler boundmass="0.5" settotalmass="10"/><worldbody><body name="b" '
                             'pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="h" '
                             'pos="1 0 0"><flexcomp name="f" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2" '
                             'rigid="true"/></body></worldbody></mujoco>'),
    "PIN_BOUNDMASS": ('<mujoco><compiler boundmass="0.5" boundinertia="0.001" settotalmass="10"/><worldbody>'
                      '<flexcomp name="f" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"><pin id="0"/>'
                      '</flexcomp></worldbody></mujoco>'),
    "FUSESTATIC_FREE": ('<mujoco><compiler fusestatic="true"/><worldbody><body name="b" pos="1 2 3"><flexcomp '
                        'name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></body></worldbody></mujoco>'),
    "DISCARDVISUAL": ('<mujoco><compiler discardvisual="true"/><default><geom contype="0" conaffinity="0"/></default>'
                      '<worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1">'
                      '<contact vertcollide="true"/></flexcomp></worldbody></mujoco>'),
}


def documents():
    for k, v in CASES.items():
        yield k, f"<mujoco><worldbody>{v}</worldbody></mujoco>"
    for k, v in CLASS_CASES.items():
        yield k, f"<mujoco>{DEFAULTS}<worldbody>{v}</worldbody></mujoco>"
    for k, v in ORDER_CASES.items():
        yield k, f"<mujoco><worldbody>{v}</worldbody></mujoco>"
    yield from DOCS.items()


def rs(x):
    return repr(float(x))


def vecs(a):
    return "&[" + ", ".join("[" + ", ".join(rs(v) for v in row) + "]" for row in a) + "]"


def floats(a):
    return "&[" + ", ".join(rs(v) for v in a) + "]"


def ints(a):
    return "&[" + ", ".join(str(int(v)) for v in a) + "]"


def name(m, kind, i):
    return mujoco.mj_id2name(m, kind, i) or ""


out = sys.stdout
out.write("// GENERATED by sim/L0/tests/scripts/gen_flexcomp_reference.py on MuJoCo "
          f"{mujoco.__version__} (unfused oracle); do not edit by hand.\n")
for key, xml in documents():
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    edges = [list(m.flex_edge[i]) for i in range(m.nflexedge)]
    names = ", ".join('"' + name(m, mujoco.mjtObj.mjOBJ_BODY, i) + '"' for i in range(m.nbody))
    flexes = ", ".join('"' + name(m, mujoco.mjtObj.mjOBJ_FLEX, i) + '"' for i in range(m.nflex))
    out.write(f"\n/// `{key}`.\nconst {key}: Expected = Expected {{\n")
    out.write(f"    doc: r#\"{xml}\"#,\n")
    out.write(f"    body_names: &[{names}],\n")
    out.write(f"    body_parent: {ints(m.body_parentid)},\n")
    out.write(f"    body_pos: {vecs(m.body_pos)},\n")
    out.write(f"    body_mass: {floats(m.body_mass)},\n")
    out.write(f"    body_inertia: {vecs(m.body_inertia)},\n")
    out.write(f"    flex_names: &[{flexes}],\n")
    out.write(f"    flex_rigid: &[{', '.join('true' if r else 'false' for r in m.flex_rigid)}],\n")
    out.write(f"    vertbody: {ints(m.flex_vertbodyid)},\n")
    out.write(f"    vert_xpos: {vecs(d.flexvert_xpos)},\n")
    out.write(f"    elem: {ints(m.flex_elem)},\n")
    out.write(f"    edges: &[{', '.join('[' + str(a) + ', ' + str(b) + ']' for a, b in edges)}],\n")
    out.write(f"    edge_rigid: &[{', '.join('true' if r else 'false' for r in m.flexedge_rigid)}],\n")
    out.write(f"    jnt_axis: {vecs(m.jnt_axis)},\n")
    out.write(f"    dof_damping: {floats(m.dof_damping)},\n")
    out.write(f"    jnt_range: {vecs(m.jnt_range)},\n")
    out.write(f"    geom_size: {floats(m.geom_size[:, 0])},\n")
    out.write(f"    geom_group: {ints(m.geom_group)},\n")
    out.write(f"    geom_contype: {ints(m.geom_contype)},\n")
    out.write(f"    neq: {m.neq},\n")
    out.write("};\n")

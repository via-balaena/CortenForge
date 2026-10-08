#!/usr/bin/env python3
"""Generate the sleep golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_sleep_reference.py \\
        sim/L0/tests/assets/golden/sleep

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes sleep.json. Its `trees` entry holds, for each model below, MuJoCo's
kinematic-tree tables (ntree, body_treeid, tree_bodyadr, tree_bodynum,
tree_dofadr, tree_dofnum, dof_treeid, tendon_treenum, tendon_treeid), the
resolved tree_sleep_policy by name, and body_awake after mj_resetData and
after one mj_step, with sleep enabled and with it disabled. The models, all
with sleep enabled:

- table: a box resting on a static box (A8's fixture);
- static_root: a static body with two hinged children;
- interleaved: static bodies between moving trees, and a static body welded
  to a moving one;
- mocap: a mocap body with a child, a free body and a static body;
- actuators: one tree per transmission (joint, jointinparent, site, body,
  slider-crank with its crank and slider sites on two trees, a fixed tendon
  over two trees) and one untouched tree;
- tendons: two trees with stiffness, two with a limit only, three with
  nothing, two with damping only, and a tendon whose wraps reach tree 3
  before tree 2;
- flex: a free body and a 2 x 2 flexcomp grid, whose four vertex bodies are
  trees. sim-mjcf does not read a `<flexcomp>` under `<worldbody>` (the
  book's Rigid-loading L47 refuses it), so the case also holds `ours_xml`,
  the same grid in the `<deformable><flexcomp>` form sim-mjcf reads, which
  the test loads;
- gravcomp: a static body and a free body with gravity compensation; also
  qfrc_gravcomp after mj_forward;
- tendon_one_tree: a stiff fixed tendon over two hinges of one tree, and a
  second tree.

Its `lengths` entry holds MuJoCo's dof_length for each model of LENGTHS (the
body sizes setStat computes at qpos0): A8's two-link chain, a free box, an
offset hinge anchor and geom, a ball joint with a hinged child, a body with
no geom whose centre of mass is its joint anchor,
and offset capsule, cylinder and ellipsoid geoms.

Its `runs` entry holds, for each run of RUNS, whether tree 0 is asleep
(`tree_asleep >= 0`) after each of `nstep` mj_step calls. A run sets fields
before a step (`set`: the step, the field, its flat index, the value): zero-g
boxes spun or moving at the tolerance, the tolerance 0 with a velocity of
+0 and of -0, A8's box_uw_negzero with qfrc_applied, or the force of
xfrc_applied, set to -0 at step 200, and A8's actuated damped hinge, whose
tree the actuator keeps awake (automatic policy never).

Its `traces` entry holds, for each trace of TRACES (A8's fixtures), after
each mj_step: tree_asleep, ncon, nefc, nisland and the callbacks that fired
(P passive, C control); and qvel, qacc, qacc_warmstart, sensordata and act
after the steps within 3 of a change of any tree's sleep state and every
25th. The
fixtures: a box resting on the plane (Euler, implicit, implicitfast, RK4, and
with islands disabled), the same box with sleep disabled, two stacked
spheres, a sphere woken by another falling on it, two spheres joined by a
connect with a third falling, a hinge on its limit with friction loss beside
a falling sphere, a box on a static table, a filtered actuator with an
allowed sleep policy, and A8's damped two-link chain started at qvel
(1.5, -1); the resting box colliding with the plane through an explicit
<pair> only; the resting box with a contact filter that keeps every pair
and logs each call (F) in the callback string; and the resting box beside a
falling sphere whose passive callback adds 0.001 times its call count to the
sphere's vertical force, so the second forward pass of the box's sleep step
computes another acceleration for the sphere than the first, and the step
advances the sphere with the first's; the filtered actuator asleep, its
ctrl set to 0.5 at step 700 (a ctrl change wakes nothing); A8's user wakes
of the resting box at step 200, by qvel, by qpos, and by a vertical force on
xfrc_applied for one step; A8's flat box rotated and spun while asleep; a
box that starts asleep (sleep="init"); the resting box with sleep
switched off at step 150 (a set of `sleep_off`); the filtered actuator on a
tendon, asleep, its ctrl changed at step 700, with actuator and tendon
sensors; the resting box with frame sensors relative to a sphere falling
toward it, relative to a static body, on the static body, between two
static bodies, and a rangefinder on the box aimed at the sphere; two
hinges joined by a fixed tendon with no stiffness, one at rest (it
sleeps), one swinging, with tendon sensors; the filtered actuator under RK4 with an accelerometer, until
it sleeps; a slider-crank whose slider
site is on the world body, its ctrl set to 1 at step 0, until its crank
sleeps; under implicitfast, a velocity-feedback actuator between a slide
that sleeps and a sprung slide that never does, the sprung one pushed at
step 50 (the sleeping actuator adds no term to the implicit derivative);
and, in zero gravity, a
sphere that starts asleep beside a resting one, which is moved against it
at step 5 while its countdown runs, so the sleeping sphere wakes with that
countdown; the resting box given a velocity of 1e-6, below the tolerance,
at step 200 (it wakes: the wake test uses a tolerance of 0); and two boxes
resting apart, asleep in two cycles, joined at step 150 by a connect made
active (`eq_active`); and, from the chunk 2c review: a limited fixed tendon
over a hinge that starts asleep and an awake one, pushed to its limit at
step 5; a joint equality and a connect, inactive, made active at step 5
between a tree that starts asleep and an awake one; a site, a
jointinparent and an adhesion actuator on trees that sleep, their ctrl set
later; a clock sensor beside a box that sleeps; three spheres in zero
gravity, one starting asleep, whose two neighbours touch it in one pass
with different countdowns; A8's filtered actuator with a delayed jointpos
and jointvel; the box that starts asleep with sleep disabled; a box
declared before the static table it rests on; two damped hanging hinges
joined by a joint equality; a pendulum beside A8's damped two-link chain,
and the same chain sprung, at a tolerance of 0.05; and the resting box
beside a falling sphere on a damped free joint, under Euler, implicit and
implicitfast. Each trace also holds `reset`: tree_asleep, ncon, nefc,
nisland and qacc after mj_makeData.

Its `islands` entry holds, for the resting box beside a hinge set onto its
limit, after mj_forward, nefc, nisland, efc_island, map_efc2iefc,
map_iefc2efc and island_nefc.

Its `refusals` entry holds the message MuJoCo raises for each model of
REFUSALS: two models whose init-asleep tree cannot sleep (A8's initmix,
joined by a connect, and initmix_contact, resting on an awake sphere), at
compile, which makes a Data; a tendon equality with sleep enabled, at
the first mj_forward; and the same with a tree that starts asleep, at
compile.

Its `tendon_equality_calls` entry holds, for that tendon equality active
and made inactive, the message each MuJoCo call raises on a fresh Data, or
null: mj_forward, mj_step, mj_step1, mj_step2, mj_forwardSkip from each
stage, mj_Euler, mjd_transitionFD and mjd_inverseFD. Its `init_timestep`
entry holds tree_asleep after mj_makeData of the box that starts asleep
with a timestep of 0 and of -0.001. Its `init_mocap` entry holds, for a
sphere that starts asleep under a mocap sphere at (0, 0, 5) in zero gravity,
with a touch sensor, ncon, the mocap body's xpos and sensordata after
mj_makeData, and ncon and sensordata after 20 steps: MuJoCo's reset runs the
init-sleep mj_forward with every mocap body at the origin.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

SLEEP = '<option><flag sleep="enable"/></option>'

TREES = {
    "table": f"""<mujoco>{SLEEP}
<worldbody><geom type="plane" size="5 5 0.1"/>
<body name="table" pos="0 0 0.5"><geom type="box" size="0.5 0.5 0.05"/></body>
<body name="b" pos="0 0 0.6495"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
</worldbody></mujoco>""",
    "static_root": f"""<mujoco>{SLEEP}
<worldbody>
<body name="base" pos="0 0 1"><geom type="box" size="0.2 0.2 0.05"/>
  <body name="arm1" pos="0.3 0 0"><joint name="h1" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/></body>
  <body name="arm2" pos="-0.3 0 0"><joint name="h2" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 -0.3 0 0" size="0.03"/></body>
</body>
</worldbody></mujoco>""",
    "interleaved": f"""<mujoco>{SLEEP}
<worldbody>
<body name="a" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/>
  <body name="a_weld" pos="0.2 0 0"><geom type="sphere" size="0.05"/></body>
</body>
<body name="s" pos="1 0 0"><geom type="box" size="0.1 0.1 0.1"/>
  <body name="s_child" pos="0 0 0.3"><geom type="box" size="0.05 0.05 0.05"/>
    <body name="c1" pos="0 0 0.3"><joint name="hc1"/>
      <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/>
      <body name="c2" pos="0.2 0 0"><joint name="hc2"/>
        <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/></body>
    </body>
  </body>
</body>
<body name="s2" pos="2 0 0"><geom type="box" size="0.1 0.1 0.1"/></body>
<body name="d" pos="3 0 1"><joint name="sd" type="slide" axis="0 0 1"/>
  <geom type="sphere" size="0.1"/></body>
</worldbody></mujoco>""",
    "mocap": f"""<mujoco>{SLEEP}
<worldbody>
<body name="m" mocap="true" pos="0 0 1">
  <geom type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/>
  <body name="m_child" pos="0 0 0.2">
    <geom type="sphere" size="0.05" contype="0" conaffinity="0"/></body>
</body>
<body name="f" pos="1 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
<body name="s" pos="2 0 0"><geom type="box" size="0.1 0.1 0.1"/></body>
</worldbody></mujoco>""",
    "actuators": f"""<mujoco>{SLEEP}
<worldbody>
<body name="b0" pos="0 0 1"><joint name="j0"/><geom type="sphere" size="0.05"/></body>
<body name="b1" pos="1 0 1"><joint name="j1"/><geom type="sphere" size="0.05"/></body>
<body name="b2" pos="2 0 1"><joint name="j2"/><geom type="sphere" size="0.05"/><site name="s2"/></body>
<body name="b3" pos="3 0 1"><joint name="j3" type="slide" axis="0 0 1"/>
  <geom type="box" size="0.05 0.05 0.05"/></body>
<body name="b4" pos="4 0 1"><joint name="j4" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.02"/><site name="crank" pos="0.1 0 0"/></body>
<body name="b5" pos="4.3 0 1"><joint name="j5" type="slide" axis="1 0 0"/>
  <geom type="sphere" size="0.05"/><site name="slider"/></body>
<body name="b6" pos="6 0 1"><joint name="j6"/><geom type="sphere" size="0.05"/></body>
<body name="b7" pos="7 0 1"><joint name="j7"/><geom type="sphere" size="0.05"/></body>
<body name="b8" pos="8 0 1"><joint name="j8"/><geom type="sphere" size="0.05"/></body>
</worldbody>
<tendon><fixed name="t67"><joint joint="j6" coef="1"/><joint joint="j7" coef="1"/></fixed></tendon>
<actuator>
  <motor joint="j0"/>
  <motor jointinparent="j1"/>
  <motor site="s2" gear="0 0 0 0 0 1"/>
  <adhesion body="b3" ctrlrange="0 1"/>
  <general cranksite="crank" slidersite="slider" cranklength="0.3"/>
  <motor tendon="t67"/>
</actuator>
</mujoco>""",
    "tendons": f"""<mujoco>{SLEEP}
<worldbody>
<body name="b0" pos="0 0 1"><joint name="j0"/><geom type="sphere" size="0.05"/><site name="s0"/></body>
<body name="b1" pos="1 0 1"><joint name="j1"/><geom type="sphere" size="0.05"/><site name="s1"/></body>
<body name="b2" pos="2 0 1"><joint name="j2"/><geom type="sphere" size="0.05"/></body>
<body name="b3" pos="3 0 1"><joint name="j3"/><geom type="sphere" size="0.05"/></body>
<body name="b4" pos="4 0 1"><joint name="j4"/><geom type="sphere" size="0.05"/></body>
<body name="b5" pos="5 0 1"><joint name="j5"/><geom type="sphere" size="0.05"/></body>
<body name="b6" pos="6 0 1"><joint name="j6"/><geom type="sphere" size="0.05"/></body>
<body name="b7" pos="7 0 1"><joint name="j7"/><geom type="sphere" size="0.05"/></body>
<body name="b8" pos="8 0 1"><joint name="j8"/><geom type="sphere" size="0.05"/></body>
</worldbody>
<tendon>
  <spatial name="stiff01" stiffness="10"><site site="s0"/><site site="s1"/></spatial>
  <fixed name="limit23" limited="true" range="-1 1">
    <joint joint="j2" coef="1"/><joint joint="j3" coef="1"/></fixed>
  <fixed name="three456"><joint joint="j4" coef="1"/><joint joint="j5" coef="1"/>
    <joint joint="j6" coef="1"/></fixed>
  <fixed name="damp78" damping="1"><joint joint="j7" coef="1"/><joint joint="j8" coef="1"/></fixed>
  <fixed name="order32"><joint joint="j3" coef="1"/><joint joint="j2" coef="1"/></fixed>
</tendon>
</mujoco>""",
    "flex": f"""<mujoco>{SLEEP}
<worldbody>
<body name="f" pos="3 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
<flexcomp name="cloth" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" pos="0 0 1" dim="2"
  radius="0.01" mass="0.1"/>
</worldbody></mujoco>""",
    "gravcomp": f"""<mujoco>{SLEEP}
<worldbody>
<body name="hover" pos="0 0 1" gravcomp="1"><geom type="box" size="0.1 0.1 0.1" mass="2"/></body>
<body name="f" pos="1 0 1" gravcomp="0.5"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody></mujoco>""",
    "tendon_one_tree": f"""<mujoco>{SLEEP}
<worldbody>
<body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/>
  <body name="a2" pos="0.2 0 0"><joint name="ja2" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/></body>
</body>
<body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0"/><geom type="sphere" size="0.05"/></body>
</worldbody>
<tendon><fixed name="t" stiffness="10"><joint joint="ja" coef="1"/><joint joint="ja2" coef="1"/></fixed></tendon>
</mujoco>""",
}

ZERO_G = '<option timestep="0.002" gravity="0 0 0"{tol}><flag sleep="enable"/></option>'
FLOAT = """<mujoco>{opt}
<worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
</worldbody></mujoco>"""
REST = """<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/>
<body name="b" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody>
<sensor><framelinvel objtype="body" objname="b"/><framepos objtype="body" objname="b"/></sensor></mujoco>"""

LENGTHS = {
    "chain2": """<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody>
<body name="l1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" damping="0.5"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/>
  <body name="l2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0" damping="0.5"/>
    <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body></body>
</worldbody></mujoco>""",
    "free_box": FLOAT.format(opt=ZERO_G.format(tol="")),
    "offset_hinge": """<mujoco><worldbody>
<body pos="0 0 1"><joint type="hinge" pos="0 0 0.3" axis="1 0 0"/>
  <geom type="sphere" size="0.1" pos="0.2 0 0" mass="1"/></body>
</worldbody></mujoco>""",
    "ball_child": """<mujoco><worldbody>
<body pos="0 0 1"><joint type="ball"/><geom type="box" size="0.05 0.1 0.2" pos="0 0 -0.2" mass="2"/>
  <body pos="0 0 -0.5"><joint type="hinge" pos="0 0.1 0" axis="0 1 0"/>
    <geom type="sphere" size="0.04" mass="0.5"/></body></body>
</worldbody></mujoco>""",
    "point_body": """<mujoco><worldbody>
<body pos="0 0 1"><joint type="hinge" axis="0 1 0"/>
  <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
</worldbody></mujoco>""",
    "offset_geoms": """<mujoco><worldbody>
<body pos="0 0 1"><joint type="hinge" axis="0 0 1"/>
  <geom type="capsule" fromto="0.1 0 0 0.4 0 0" size="0.02"/>
  <geom type="cylinder" size="0.05 0.1" pos="0 0.3 0"/>
  <geom type="ellipsoid" size="0.05 0.1 0.15" pos="0 0 0.25"/></body>
</worldbody></mujoco>""",
}

# name: (xml, nstep, [(step, field, flat index, value)])
RUNS = {
    "spin": (FLOAT.format(opt=ZERO_G.format(tol="")), 40, [(0, "qvel", 5, 3e-4)]),
    "at_tolerance": (FLOAT.format(opt=ZERO_G.format(tol="")), 40, [(0, "qvel", 0, 1e-4)]),
    "tol0_zero": (FLOAT.format(opt=ZERO_G.format(tol=' sleep_tolerance="0"')), 40, []),
    "tol0_negzero": (FLOAT.format(opt=ZERO_G.format(tol=' sleep_tolerance="0"')), 40,
                     [(0, "qvel", 0, -0.0)]),
    "negzero_qfrc": (REST, 600, [(200, "qfrc_applied", 2, -0.0)]),
    "negzero_xfrc": (REST, 600, [(200, "xfrc_applied", 6 + 2, -0.0)]),
    "actuated": ("""<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody>
<body name="l1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody>
<actuator><general name="a1" joint="j1" dyntype="filter" dynprm="0.05" gainprm="1" biastype="affine"
  biasprm="0 -20 -1"/></actuator>
</mujoco>""", 800, []),
}

PLANE = '<geom type="plane" size="5 5 0.1"/>'
BOX = '<body name="b" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>'
BOX_SENSORS = ('<sensor><framelinvel objtype="body" objname="b"/>'
               '<framepos objtype="body" objname="b"/></sensor>')


def box_rest(opt='', flags=''):
    return (f'<mujoco><option timestep="0.002"{opt}><flag sleep="enable"{flags}/></option>\n'
            f'<worldbody>{PLANE}\n{BOX}</worldbody>\n{BOX_SENSORS}</mujoco>')


SPHERE = '<body name="{n}" pos="{p}"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>'
SLEEP_OPT = '<option timestep="0.002"><flag sleep="enable"/></option>'

TRACES_ACT_SLEEP = f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="l1" pos="0 0 1" sleep="allowed"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody>
<actuator><general name="a1" joint="j1" dyntype="filter" dynprm="0.05" gainprm="1" biastype="affine"
  biasprm="0 -20 -1"/></actuator>
<sensor><actuatorfrc actuator="a1"/><jointvel joint="j1"/></sensor></mujoco>"""
TRACES_ACT_SLEEP_TENDON = f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="l1" pos="0 0 1" sleep="allowed"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody>
<tendon><fixed name="t1"><joint joint="j1" coef="2"/></fixed></tendon>
<actuator><general name="a1" tendon="t1" dyntype="filter" dynprm="0.05" gainprm="1" biastype="affine"
  biasprm="0 -20 -1"/></actuator>
<sensor><actuatorfrc actuator="a1"/><actuatorpos actuator="a1"/><tendonpos tendon="t1"/>
  <tendonvel tendon="t1"/><jointvel joint="j1"/></sensor></mujoco>"""
TRACES_ACT_SLEEP_RK4 = """<mujoco><option timestep="0.002" integrator="RK4"><flag sleep="enable"/></option>
<worldbody>
<body name="l1" pos="0 0 1" sleep="allowed"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/><site name="s1" pos="0.3 0 0"/></body>
</worldbody>
<actuator><general name="a1" joint="j1" dyntype="filter" dynprm="0.05" gainprm="1" biastype="affine"
  biasprm="0 -20 -1"/></actuator>
<sensor><actuatorfrc actuator="a1"/><accelerometer site="s1"/><jointvel joint="j1"/></sensor></mujoco>"""
TRACES_SLIDER_STATIC = f"""<mujoco>{SLEEP_OPT}
<worldbody>
<site name="s" pos="0.3 0 1"/>
<body name="crank" pos="0 0 1" sleep="allowed"><joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
  <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.02" mass="1"/>
  <site name="c" pos="0.1 0 0"/></body>
</worldbody>
<actuator><general name="a" cranksite="c" slidersite="s" cranklength="0.5" gainprm="1"/></actuator>
<sensor><actuatorfrc actuator="a"/><jointpos joint="j"/><actuatorpos actuator="a"/></sensor></mujoco>"""

# name: (xml, nstep, sets, sleep enabled[, log the contact filter[, (dof, scale) of the passive counter]])
TRACES = {
    "box_rest": (box_rest(), 300, [], True),
    "box_rest_nosleep": (box_rest(), 5, [], False),
    "box_rest_implicit": (box_rest(' integrator="implicit"'), 300, [], True),
    "box_rest_implicitfast": (box_rest(' integrator="implicitfast"'), 300, [], True),
    "box_rest_RK4": (box_rest(' integrator="RK4"'), 300, [], True),
    "box_noisland": (box_rest('', ' island="disable"'), 300, [], True),
    "sstack": (f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
{SPHERE.format(n="a", p="0 0 0.0995")}
{SPHERE.format(n="b", p="0 0 0.2985")}</worldbody>
<sensor><framelinvel objtype="body" objname="b"/></sensor></mujoco>""", 300, [], True),
    "swake": (f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
{SPHERE.format(n="a", p="0 0 0.0995")}
{SPHERE.format(n="b", p="0 0 0.6")}</worldbody>
<sensor><framelinvel objtype="body" objname="a"/></sensor></mujoco>""", 400, [], True),
    "eqpair": (f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
{SPHERE.format(n="a", p="0 0 0.0995")}
{SPHERE.format(n="b", p="0.3 0 0.0995")}
{SPHERE.format(n="c", p="1.0 0 0.6")}</worldbody>
<equality><connect body1="a" body2="b" anchor="0.15 0 0"/></equality>
<sensor><framelinvel objtype="body" objname="b"/></sensor></mujoco>""", 400, [], True),
    "limit": (f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="l1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" damping="0.3" limited="true"
  range="-30 30" frictionloss="0.01"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
<body name="f" pos="1 0 1"><freejoint/><geom type="sphere" size="0.05" mass="0.2"/></body>
</worldbody>
<sensor><jointvel joint="j1"/></sensor></mujoco>""", 400, [], True),
    "table": (TREES["table"], 300, [], True),
    "act_sleep": (TRACES_ACT_SLEEP, 800, [], True),
    "act_sleep_ctrl": (TRACES_ACT_SLEEP, 800, [(700, "ctrl", 0, 0.5)], True),
    "act_tendon_ctrl": (TRACES_ACT_SLEEP_TENDON, 800, [(700, "ctrl", 0, 0.5)], True),
    "act_sleep_RK4": (TRACES_ACT_SLEEP_RK4, 700, [], True),
    "slider_static": (TRACES_SLIDER_STATIC, 1150, [(0, "ctrl", 0, 1.0)], True),
    "deriv_asleep": ("""<mujoco><option timestep="0.002" integrator="implicitfast"><flag sleep="enable"/></option>
<worldbody>
<body name="a" pos="0 0 1" sleep="allowed"><joint name="ja" type="slide" axis="1 0 0" damping="5"/>
  <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/><site name="sa"/></body>
<body name="b" pos="1 0 1" sleep="never"><joint name="jb" type="slide" axis="1 0 0" stiffness="10"/>
  <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/><site name="sb"/></body>
</worldbody>
<actuator><general name="act" site="sa" refsite="sb" gear="1 0 0 0 0 0" biastype="affine" biasprm="0 0 -50"/></actuator>
</mujoco>""", 80, [(50, "qvel", 1, 1.0)], True),
    "sensor_mix": (f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
<body name="b" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/>
  <site name="up" pos="0 0 0.1" zaxis="0 0 1"/></body>
<body name="f" pos="0 0 5"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
<body name="s" pos="-2 0 0.5"><geom type="box" size="0.1 0.1 0.1"/></body>
<body name="s2" pos="-3 0 0.5"><geom type="box" size="0.1 0.1 0.1"/></body></worldbody>
<sensor><framepos objtype="body" objname="b" reftype="body" refname="f"/>
  <framepos objtype="body" objname="b" reftype="body" refname="s"/>
  <framepos objtype="body" objname="s"/>
  <framelinvel objtype="body" objname="f" reftype="body" refname="b"/>
  <framepos objtype="body" objname="s" reftype="body" refname="s2"/>
  <rangefinder site="up"/></sensor></mujoco>""",
                   200, [], True),
    "two_tree_tendon": (f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="l1" pos="0 0 1"><joint name="j1" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body>
<body name="l2" pos="1 0 1"><joint name="j2" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body>
</worldbody>
<tendon><fixed name="t"><joint joint="j1" coef="1"/><joint joint="j2" coef="1"/></fixed></tendon>
<sensor><tendonpos tendon="t"/><tendonvel tendon="t"/></sensor></mujoco>""",
                        300, [(0, "qvel", 1, 2.0)], True),
    "chain2": (LENGTHS["chain2"], 3000, [(0, "qvel", 0, 1.5), (0, "qvel", 1, -1.0)], True),
    "box_rest_pair": ("""<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom name="floor" type="plane" size="5 5 0.1" contype="0" conaffinity="0"/>
<body name="b" pos="0 0 0.0995"><freejoint/>
  <geom name="box" type="box" size="0.1 0.1 0.1" mass="1" contype="0" conaffinity="0"/></body></worldbody>
<contact><pair geom1="floor" geom2="box"/></contact>
<sensor><framelinvel objtype="body" objname="b"/><framepos objtype="body" objname="b"/></sensor></mujoco>""",
                      300, [], True),
    "box_rest_filter": (box_rest(), 120, [], True, True),
    "box_rest_counter": (f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
{BOX}
<body name="s" pos="2 0 5"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody>
<sensor><framelinvel objtype="body" objname="s"/></sensor></mujoco>""", 100, [], True, False, (8, 1e-3)),
}

UW = box_rest()
RBOX = """<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/>
<body name="b" pos="0 0 0.0595"><freejoint/><geom type="box" size="0.1 0.08 0.06" mass="1"/></body></worldbody>
<sensor><framelinvel objtype="body" objname="b"/><frameangvel objtype="body" objname="b"/></sensor></mujoco>"""
BOX_INIT = """<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/>
<body name="b" pos="0 0 0.0995" sleep="init"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
</worldbody>
<sensor><framelinvel objtype="body" objname="b"/><framepos objtype="body" objname="b"/></sensor></mujoco>"""
TRACES.update({
    "uw_qvel": (UW, 300, [(200, "qvel", 0, 0.01)], True),
    "uw_qpos": (UW, 300, [(200, "qpos", 0, 0.05)], True),
    "uw_xfrc": (UW, 300, [(200, "xfrc_applied", 8, 3.0), (201, "xfrc_applied", 8, 0.0)], True),
    "rbox": (RBOX, 300, [(200, "qpos", 3, 0.9659258262890683), (200, "qpos", 6, 0.25881904510252074),
                         (200, "qvel", 5, 0.5)], True),
    "box_init": (BOX_INIT, 60, [], True),
    "sleep_off": (UW, 200, [(150, "sleep_off", 0, 0.0)], True),
    "uw_qvel_small": (UW, 300, [(200, "qvel", 0, 1e-6)], True),
    "eq_activate": (f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
<body name="a" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
<body name="b" pos="1 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody>
<equality><connect body1="a" body2="b" anchor="0.5 0 0.0995" active="false"/></equality></mujoco>""",
                    300, [(150, "eq_active", 0, 1.0)], True),
    "wake_countdown": ("""<mujoco><option timestep="0.002" gravity="0 0 0"><flag sleep="enable"/></option>
<worldbody>
<body name="a" pos="0 0 1" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
<body name="b" pos="0.5 0 1"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody></mujoco>""", 60, [(5, "qpos", 7, 0.199)], True),
})

ZG = '<option timestep="0.002" gravity="0 0 0"><flag sleep="enable"/></option>'
HINGES = """<body name="l1" pos="0 0 1" sleep="init"><joint name="j1" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body>
<body name="l2" pos="1 0 1" sleep="allowed"><joint name="j2" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body>"""
ACT = 'dyntype="filter" dynprm="0.05" gainprm="1" biastype="affine" biasprm="0 -20 -1"'
CHAIN = """<body name="p" pos="2 0 1"><joint name="jp" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body>
<body name="l1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" damping="0.5"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/>
  <body name="l2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0" damping="0.5"{spring}/>
    <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body></body>"""
DAMPED_SPHERE = """<mujoco><option timestep="0.002" integrator="{integ}"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/>
<body name="b" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
<body name="s" pos="2 0 2"><joint type="free" damping="0.1"/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody></mujoco>"""
TRACES.update({
    "tendon_wake": (f"""<mujoco>{ZG}<worldbody>{HINGES}</worldbody>
<tendon><fixed name="t" limited="true" range="-0.2 0.2"><joint joint="j1" coef="1"/><joint joint="j2" coef="1"/></fixed></tendon>
</mujoco>""", 10, [(5, "qpos", 1, 0.25)], True),
    "joint_eq_wake": (f"""<mujoco>{ZG}<worldbody>{HINGES}</worldbody>
<equality><joint joint1="j1" joint2="j2" active="false"/></equality></mujoco>""",
                      10, [(5, "eq_active", 0, 1.0)], True),
    "connect_eq_wake": (f"""<mujoco>{ZG}<worldbody>
<body name="a" pos="0 0 1" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
<body name="b" pos="0.5 0 1"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody>
<equality><connect body1="a" body2="b" anchor="0.25 0 0" active="false"/></equality></mujoco>""",
                        10, [(5, "eq_active", 0, 1.0)], True),
    "site_act_sleep": (f"""<mujoco>{SLEEP_OPT}<worldbody>
<body name="l1" pos="0 0 1" sleep="allowed"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/><site name="s"/></body></worldbody>
<actuator><general name="a1" site="s" gear="0 0 0 0 1 0" {ACT}/></actuator>
<sensor><actuatorfrc actuator="a1"/></sensor></mujoco>""", 200, [(150, "ctrl", 0, 0.5)], True),
    "jip_act_sleep": (f"""<mujoco>{SLEEP_OPT}<worldbody>
<body name="l1" pos="0 0 1" sleep="allowed"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body></worldbody>
<actuator><general name="a1" jointinparent="j1" {ACT}/></actuator>
<sensor><actuatorfrc actuator="a1"/></sensor></mujoco>""", 760, [(700, "ctrl", 0, 0.5)], True),
    "adhesion_sleep": (f"""<mujoco>{SLEEP_OPT}<worldbody>{PLANE}
<body name="b" pos="0 0 0.0995" sleep="allowed"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
</worldbody>
<actuator><adhesion name="a1" body="b" ctrlrange="0 1" gain="5"/></actuator>
<sensor><actuatorfrc actuator="a1"/></sensor></mujoco>""", 200, [(150, "ctrl", 0, 0.5)], True),
    "clock_sleep": (f"""<mujoco>{SLEEP_OPT}<worldbody>{PLANE}
<body name="b" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
</worldbody><sensor><clock/></sensor></mujoco>""", 120, [], True),
    "wake_lower": (f"""<mujoco>{ZG}<worldbody>
<body name="s" pos="0 0 1" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
<body name="a" pos="-0.5 0 1"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
<body name="b" pos="0.5 0 1"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody></mujoco>""", 8, [(1, "qvel", 6, 0.01), (2, "qvel", 6, 0.0), (5, "qpos", 7, -0.199),
                              (5, "qpos", 14, 0.199)], True),
    "delayed_sleep": (f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="l1" pos="0 0 1" sleep="allowed"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody>
<actuator><general name="a1" joint="j1" {ACT}/></actuator>
<sensor><jointpos joint="j1" delay="0.02" nsample="20"/><jointvel joint="j1" delay="0.02" nsample="20"/></sensor>
</mujoco>""", 660, [], True),
    "box_init_nosleep": (BOX_INIT, 5, [], False),
    "box_on_table_first": (f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="b" pos="0 0 0.6495"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
<body name="table" pos="0 0 0.5"><geom type="box" size="0.5 0.5 0.05"/></body>
</worldbody></mujoco>""", 100, [], True),
    "joint_eq_sleep": (f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="2"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body>
<body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="2"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body>
</worldbody>
<equality><joint joint1="ja" joint2="jb"/></equality></mujoco>""", 1170, [(0, "qvel", 0, 0.5)], True),
    "chain_beside_pendulum": (f"""<mujoco>{SLEEP_OPT}<worldbody>{CHAIN.format(spring="")}</worldbody></mujoco>""",
                              2800, [(0, "qvel", 1, 1.5), (0, "qvel", 2, -1.0)], True),
    "chain_sprung": ("""<mujoco><option timestep="0.002" sleep_tolerance="0.05"><flag sleep="enable"/></option>
<worldbody>""" + CHAIN.format(spring=' stiffness="20" springref="1.2"') + "</worldbody></mujoco>",
                     130, [(0, "qvel", 1, 1.5), (0, "qvel", 2, -1.0)], True),
    "damped_sphere_Euler": (DAMPED_SPHERE.format(integ="Euler"), 100, [], True),
    "damped_sphere_implicit": (DAMPED_SPHERE.format(integ="implicit"), 100, [], True),
    "damped_sphere_implicitfast": (DAMPED_SPHERE.format(integ="implicitfast"), 100, [], True),
})

ISLANDS = f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
<body name="b" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
<body name="h" pos="2 0 1"><joint name="jh" axis="0 1 0" limited="true" range="-0.5 0.5"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody></mujoco>"""


def islands():
    m = mujoco.MjModel.from_xml_string(ISLANDS)
    d = mujoco.MjData(m)
    d.qpos[7] = 0.6
    mujoco.mj_forward(m, d)
    return {"xml": ISLANDS, "qpos": [[7, 0.6]], "nefc": int(d.nefc), "nisland": int(d.nisland),
            "efc_island": ints(d.efc_island[:d.nefc]), "map_efc2iefc": ints(d.map_efc2iefc[:d.nefc]),
            "map_iefc2efc": ints(d.map_iefc2efc[:d.nefc]), "island_nefc": ints(d.island_nefc[:d.nisland])}


REFUSALS = {
    "initmix": f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
<body name="a" pos="0 0 0.0995" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
{SPHERE.format(n="b", p="0.3 0 0.0995")}
{SPHERE.format(n="c", p="1.0 0 0.6")}</worldbody>
<equality><connect body1="a" body2="b" anchor="0.15 0 0"/></equality></mujoco>""",
    "initmix_contact": f"""<mujoco>{SLEEP_OPT}
<worldbody>{PLANE}
{SPHERE.format(n="a", p="0 0 0.0995")}
<body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody></mujoco>""",
    "tendon_equality": f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="a" pos="0 0 1"><joint name="ja"/><geom type="sphere" size="0.05" mass="1"/></body>
<body name="b" pos="1 0 1"><joint name="jb"/><geom type="sphere" size="0.05" mass="1"/></body>
</worldbody>
<tendon><fixed name="t"><joint joint="ja" coef="1"/><joint joint="jb" coef="-1"/></fixed></tendon>
<equality><tendon tendon1="t"/></equality></mujoco>""",
    "init_tendon_equality": f"""<mujoco>{SLEEP_OPT}
<worldbody>
<body name="a" pos="0 0 1" sleep="init"><joint name="ja"/><geom type="sphere" size="0.05" mass="1"/></body>
<body name="b" pos="1 0 1"><joint name="jb"/><geom type="sphere" size="0.05" mass="1"/></body>
</worldbody>
<tendon><fixed name="t"><joint joint="ja" coef="1"/><joint joint="jb" coef="-1"/></fixed></tendon>
<equality><tendon tendon1="t"/></equality></mujoco>""",
}

POLICY = {
    int(mujoco.mjtSleepPolicy.mjSLEEP_AUTO): "Auto",
    int(mujoco.mjtSleepPolicy.mjSLEEP_AUTO_NEVER): "AutoNever",
    int(mujoco.mjtSleepPolicy.mjSLEEP_AUTO_ALLOWED): "AutoAllowed",
    int(mujoco.mjtSleepPolicy.mjSLEEP_NEVER): "Never",
    int(mujoco.mjtSleepPolicy.mjSLEEP_ALLOWED): "Allowed",
    int(mujoco.mjtSleepPolicy.mjSLEEP_INIT): "Init",
}


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def floats(a):
    return [float(x) for x in a]


def awake_after(m, step):
    d = mujoco.MjData(m)
    if step:
        mujoco.mj_step(m, d)
    return ints(d.body_awake)


OURS_XML = {
    "flex": """<mujoco><option><flag sleep="enable"/></option>
<worldbody>
<body name="f" pos="3 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
</worldbody>
<deformable><flexcomp name="cloth" type="grid" count="2 2 1" spacing="0.1" pos="0 0 1" dim="2"
  radius="0.01" mass="0.1"/></deformable>
</mujoco>""",
}


def tree_case(name, xml):
    m = mujoco.MjModel.from_xml_string(xml)
    out = {
        "name": name,
        "xml": xml,
        "ours_xml": OURS_XML.get(name),
        "ntree": int(m.ntree),
        "body_treeid": ints(m.body_treeid),
        "tree_bodyadr": ints(m.tree_bodyadr),
        "tree_bodynum": ints(m.tree_bodynum),
        "tree_dofadr": ints(m.tree_dofadr),
        "tree_dofnum": ints(m.tree_dofnum),
        "dof_treeid": ints(m.dof_treeid),
        "tendon_treenum": ints(m.tendon_treenum),
        "tendon_treeid": ints(m.tendon_treeid),
        "tree_sleep_policy": [POLICY[int(p)] for p in m.tree_sleep_policy],
        "body_awake_reset": awake_after(m, False),
        "body_awake_step": awake_after(m, True),
    }
    m.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    out["body_awake_reset_nosleep"] = awake_after(m, False)
    out["body_awake_step_nosleep"] = awake_after(m, True)
    m.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    out["qfrc_gravcomp"] = floats(d.qfrc_gravcomp)
    return out


def length_case(name, xml):
    m = mujoco.MjModel.from_xml_string(xml)
    return {"name": name, "xml": xml, "dof_length": floats(m.dof_length)}


def run_case(name, xml, nstep, sets):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    asleep = []
    for k in range(nstep):
        for at, field, idx, value in sets:
            if at == k:
                getattr(d, field).reshape(-1)[idx] = value
        mujoco.mj_step(m, d)
        asleep.append(bool(d.tree_asleep[0] >= 0))
    return {"name": name, "xml": xml, "sets": [list(x) for x in sets], "asleep": asleep}


def trace_case(name, xml, nstep, sets, sleep, log_filter=False, passive_force=None):
    m = mujoco.MjModel.from_xml_string(xml)
    if not sleep:
        m.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    d = mujoco.MjData(m)
    reset = {"tree_asleep": ints(d.tree_asleep), "ncon": int(d.ncon), "nefc": int(d.nefc),
             "nisland": int(d.nisland), "qacc": floats(d.qacc)}
    log = []
    calls = [0]

    def passive(m_, d_):
        log.append("P")
        if passive_force is not None:
            calls[0] += 1
            d_.qfrc_passive[passive_force[0]] += passive_force[1] * calls[0]

    mujoco.set_mjcb_passive(passive)
    mujoco.set_mjcb_control(lambda m_, d_: log.append("C"))
    if log_filter:
        mujoco.set_mjcb_contactfilter(lambda m_, d_, g1, g2: log.append("F") or 0)
    steps = []
    try:
        for k in range(nstep):
            for at, field, idx, value in sets:
                if at == k and field == "sleep_off":
                    m.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
                elif at == k:
                    getattr(d, field).reshape(-1)[idx] = value
            log.clear()
            mujoco.mj_step(m, d)
            steps.append({"tree_asleep": ints(d.tree_asleep), "ncon": int(d.ncon), "nefc": int(d.nefc),
                          "nisland": int(d.nisland), "cb": "".join(log),
                          "qvel": floats(d.qvel), "qacc": floats(d.qacc),
                          "qacc_warmstart": floats(d.qacc_warmstart), "sensordata": floats(d.sensordata),
                          "act": floats(d.act)})
    finally:
        mujoco.set_mjcb_passive(None)
        mujoco.set_mjcb_control(None)
        mujoco.set_mjcb_contactfilter(None)
    asleep = [[a >= 0 for a in st["tree_asleep"]] for st in steps]
    change = {k for k in range(nstep) if asleep[k] != (asleep[k - 1] if k else [False] * m.ntree)}
    keep = {k for c in change for k in range(c - 3, c + 4) if 0 <= k < nstep} | set(range(0, nstep, 25))
    for k, st in enumerate(steps):
        if k not in keep:
            for f in ("qvel", "qacc", "qacc_warmstart", "sensordata", "act"):
                del st[f]
    return {"name": name, "xml": xml, "sets": [list(x) for x in sets], "sleep": sleep,
            "log_filter": log_filter, "passive_force": list(passive_force) if passive_force else None,
            "reset": reset, "steps": steps}


def refusal(xml):
    try:
        m = mujoco.MjModel.from_xml_string(xml)
        mujoco.mj_forward(m, mujoco.MjData(m))
    except Exception as e:  # noqa: BLE001 — MuJoCo's mjERROR, recorded as its message
        return str(e)
    return None


TENDON_EQUALITY_CALLS = {
    "mj_forward": lambda m, d: mujoco.mj_forward(m, d),
    "mj_step": lambda m, d: mujoco.mj_step(m, d),
    "mj_step1": lambda m, d: mujoco.mj_step1(m, d),
    "mj_step2": lambda m, d: mujoco.mj_step2(m, d),
    "mj_forwardSkip(NONE)": lambda m, d: mujoco.mj_forwardSkip(m, d, mujoco.mjtStage.mjSTAGE_NONE, 0),
    "mj_forwardSkip(POS)": lambda m, d: mujoco.mj_forwardSkip(m, d, mujoco.mjtStage.mjSTAGE_POS, 0),
    "mj_forwardSkip(VEL)": lambda m, d: mujoco.mj_forwardSkip(m, d, mujoco.mjtStage.mjSTAGE_VEL, 0),
    "mj_Euler": lambda m, d: mujoco.mj_Euler(m, d),
    "mjd_transitionFD": lambda m, d: mujoco.mjd_transitionFD(
        m, d, 1e-6, 0, np.zeros((2 * m.nv, 2 * m.nv)), None, None, None),
    "mjd_inverseFD": lambda m, d: mujoco.mjd_inverseFD(
        m, d, 1e-6, 0, np.zeros((m.nv, m.nv)), None, None, None, None, None, None),
}


def tendon_equality_calls():
    out = {}
    active = REFUSALS["tendon_equality"]
    inactive = active.replace('<tendon tendon1="t"/>', '<tendon tendon1="t" active="false"/>')
    assert inactive != active
    for key, xml in (("active", active), ("inactive", inactive)):
        calls = {}
        for name, call in TENDON_EQUALITY_CALLS.items():
            m = mujoco.MjModel.from_xml_string(xml)
            try:
                call(m, mujoco.MjData(m))
                calls[name] = None
            except Exception as e:  # noqa: BLE001 — MuJoCo's mjERROR, recorded as its message
                calls[name] = str(e)
        out[key] = {"xml": xml, "calls": calls}
    return out


INIT_MOCAP = """<mujoco><option timestep="0.002" gravity="0 0 0"><flag sleep="enable"/></option>
<worldbody>
<body name="m" mocap="true" pos="0 0 5"><geom type="sphere" size="0.2"/></body>
<body name="b" pos="0 0 0.1" sleep="init"><freejoint/><geom type="sphere" size="0.15" mass="1"/>
  <site name="t" size="0.3"/></body>
</worldbody>
<sensor><touch site="t"/></sensor></mujoco>"""


def init_mocap():
    m = mujoco.MjModel.from_xml_string(INIT_MOCAP)
    d = mujoco.MjData(m)
    out = {"xml": INIT_MOCAP, "ncon": int(d.ncon), "mocap_xpos": floats(d.xpos[1]),
           "sensordata": floats(d.sensordata), "tree_asleep": ints(d.tree_asleep)}
    for _ in range(20):
        mujoco.mj_step(m, d)
    out["after_20"] = {"ncon": int(d.ncon), "sensordata": floats(d.sensordata)}
    return out


def init_timestep():
    out = {}
    for dt in (0.0, -0.001):
        m = mujoco.MjModel.from_xml_string(BOX_INIT)
        m.opt.timestep = dt
        out[repr(dt)] = ints(mujoco.MjData(m).tree_asleep)
    return out


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    trees = [tree_case(name, xml) for name, xml in TREES.items()]
    lengths = [length_case(name, xml) for name, xml in LENGTHS.items()]
    runs = [run_case(name, *run) for name, run in RUNS.items()]
    traces = [trace_case(name, *trace) for name, trace in TRACES.items()]
    refusals = {name: refusal(xml) for name, xml in REFUSALS.items()}
    doc = {"oracle": marker, "trees": finite_or_string(trees), "lengths": finite_or_string(lengths),
           "runs": finite_or_string(runs)}
    write(os.path.join(sys.argv[1], "sleep_traces.json"), {"oracle": marker, "traces": finite_or_string(traces)})
    doc["refusals"] = [{"name": name, "xml": REFUSALS[name], "message": msg} for name, msg in refusals.items()]
    doc["tendon_equality_calls"] = tendon_equality_calls()
    doc["init_timestep"] = init_timestep()
    doc["init_mocap"] = init_mocap()
    doc["islands"] = islands()
    write(os.path.join(sys.argv[1], "sleep.json"), doc)
    print(f"{len(trees)} tree, {len(lengths)} length, {len(runs)} run and {len(traces)} trace cases"
          f" -> {sys.argv[1]}")


if __name__ == "__main__":
    main()

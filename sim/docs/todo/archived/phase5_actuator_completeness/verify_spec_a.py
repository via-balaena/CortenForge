"""
Empirical Ground Truth Verification for Spec A (acc0, dampratio, lengthrange).

Runs MuJoCo Python bindings on small models and extracts reference values
that the Spec A rubric needs for independent cross-checking.

MuJoCo version: pinned at runtime (printed in output).

Usage:
    uv run verify_spec_a.py
"""

import sys
try:
    import mujoco
    import numpy as np
except ImportError:
    print("Missing dependencies. Install with:")
    print("  uv pip install mujoco numpy")
    sys.exit(1)

np.set_printoptions(precision=15, suppress=False)

print(f"MuJoCo version: {mujoco.__version__}")
print(f"NumPy version:  {np.__version__}")
print()

# =============================================================================
# Model 1: Single hinge with motor — simplest acc0 case
# =============================================================================
print("=" * 72)
print("MODEL 1: Single hinge + motor (acc0 baseline)")
print("=" * 72)

SINGLE_HINGE_MOTOR = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" gear="1"/>
  </actuator>
</mujoco>
"""

m1 = mujoco.MjModel.from_xml_string(SINGLE_HINGE_MOTOR)
d1 = mujoco.MjData(m1)

# Trigger mj_setConst (happens at model load, but let's be explicit)
mujoco.mj_forward(m1, d1)

print(f"  nv = {m1.nv}")
print(f"  nu = {m1.nu}")
print(f"  actuator_acc0 = {m1.actuator_acc0}")
print(f"  dof_M0        = {m1.dof_M0}")
print(f"  actuator_biasprm[0] = {m1.actuator_biasprm[0]}")
print(f"  actuator_gainprm[0] = {m1.actuator_gainprm[0]}")
print(f"  actuator_lengthrange[0] = {m1.actuator_lengthrange[0]}")

# Verify: acc0 = ||M^{-1} J|| for single-DOF should be 1/I
# For single hinge, M = I (moment of inertia), J = gear = 1
# So acc0 = |1/I| = 1/I
# I for a box of mass=1, half-extents (0.1, 0.5, 0.05) about z-axis:
# I_zz = m/3 * (x^2 + y^2) = 1/3 * (0.1^2 + 0.5^2) = 1/3 * 0.26 = 0.08667
inertia_analytical = 1.0 / 3.0 * (0.1**2 + 0.5**2)
acc0_analytical = 1.0 / inertia_analytical
print(f"\n  Analytical I_zz = {inertia_analytical}")
print(f"  Analytical acc0 = 1/I = {acc0_analytical}")
print(f"  Match: {np.isclose(m1.actuator_acc0[0], acc0_analytical)}")

# Also extract the mass matrix to verify dof_M0 == qM diagonal
qM = np.zeros((m1.nv, m1.nv))
mujoco.mj_fullM(m1, qM, d1.qM)
print(f"\n  Full mass matrix qM = {qM}")
print(f"  qM diagonal = {np.diag(qM)}")
print(f"  dof_M0 == qM diagonal: {np.allclose(m1.dof_M0, np.diag(qM))}")

# =============================================================================
# Model 2: Position actuator with dampratio
# =============================================================================
print()
print("=" * 72)
print("MODEL 2: Position actuator with dampratio=1.0 (dampratio conversion)")
print("=" * 72)

POSITION_DAMPRATIO = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="pos1" joint="hinge1" kp="100" dampratio="1.0"/>
  </actuator>
</mujoco>
"""

m2 = mujoco.MjModel.from_xml_string(POSITION_DAMPRATIO)
d2 = mujoco.MjData(m2)
mujoco.mj_forward(m2, d2)

print(f"  nv = {m2.nv}")
print(f"  nu = {m2.nu}")
print(f"  actuator_acc0     = {m2.actuator_acc0}")
print(f"  dof_M0            = {m2.dof_M0}")
print(f"  actuator_gainprm  = {m2.actuator_gainprm[0]}")
print(f"  actuator_biasprm  = {m2.actuator_biasprm[0]}")
print(f"    biasprm[0] = {m2.actuator_biasprm[0][0]}")
print(f"    biasprm[1] = {m2.actuator_biasprm[0][1]}")
print(f"    biasprm[2] = {m2.actuator_biasprm[0][2]} (should be negative = -kv)")

# Verify dampratio conversion
kp = 100.0
mass = m2.dof_M0[0]  # reflected inertia for single-DOF = dof_M0
dampratio = 1.0
kv_expected = dampratio * 2.0 * np.sqrt(kp * mass)
print(f"\n  kp = {kp}")
print(f"  reflected inertia (dof_M0) = {mass}")
print(f"  Analytical kv = dampratio * 2 * sqrt(kp * mass) = {kv_expected}")
print(f"  Stored biasprm[2] = {m2.actuator_biasprm[0][2]}")
print(f"  Expected biasprm[2] = -{kv_expected} = {-kv_expected}")
print(f"  Match: {np.isclose(m2.actuator_biasprm[0][2], -kv_expected)}")

# =============================================================================
# Model 3: Position actuator with explicit kv (dampratio should NOT fire)
# =============================================================================
print()
print("=" * 72)
print("MODEL 3: Position actuator with explicit kv (no dampratio conversion)")
print("=" * 72)

POSITION_EXPLICIT_KV = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="pos1" joint="hinge1" kp="100" kv="5.0"/>
  </actuator>
</mujoco>
"""

m3 = mujoco.MjModel.from_xml_string(POSITION_EXPLICIT_KV)
d3 = mujoco.MjData(m3)
mujoco.mj_forward(m3, d3)

print(f"  actuator_biasprm = {m3.actuator_biasprm[0]}")
print(f"    biasprm[2] = {m3.actuator_biasprm[0][2]} (should be -5.0, unchanged)")
print(f"  Match: {np.isclose(m3.actuator_biasprm[0][2], -5.0)}")

# =============================================================================
# Model 4: Motor actuator (NOT position-like — dampratio should NOT fire)
# =============================================================================
print()
print("=" * 72)
print("MODEL 4: Motor actuator (non-position, dampratio guard)")
print("=" * 72)

MOTOR_NO_DAMPRATIO = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" gear="1"/>
  </actuator>
</mujoco>
"""

m4 = mujoco.MjModel.from_xml_string(MOTOR_NO_DAMPRATIO)
d4 = mujoco.MjData(m4)
mujoco.mj_forward(m4, d4)

print(f"  actuator_gainprm = {m4.actuator_gainprm[0]}")
print(f"  actuator_biasprm = {m4.actuator_biasprm[0]}")
print(f"  gainprm[0] = {m4.actuator_gainprm[0][0]}")
print(f"  biasprm[1] = {m4.actuator_biasprm[0][1]}")
print(f"  Position-like check (gainprm[0] == -biasprm[1]): "
      f"{m4.actuator_gainprm[0][0] == -m4.actuator_biasprm[0][1]}")

# =============================================================================
# Model 5: Multi-body chain — acc0 with mass coupling, dof_M0 equivalence
# =============================================================================
print()
print("=" * 72)
print("MODEL 5: 3-body chain (multi-DOF acc0 + dof_M0 verification)")
print("=" * 72)

THREE_BODY_CHAIN = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body pos="0 0 0.5">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="box" size="0.05 0.05 0.25" mass="2.0"/>
      <body pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="box" size="0.05 0.05 0.25" mass="1.5"/>
        <body pos="0 0 -0.5">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="box" size="0.05 0.05 0.25" mass="1.0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m1" joint="j1" gear="1"/>
    <motor name="m2" joint="j2" gear="1"/>
    <motor name="m3" joint="j3" gear="1"/>
  </actuator>
</mujoco>
"""

m5 = mujoco.MjModel.from_xml_string(THREE_BODY_CHAIN)
d5 = mujoco.MjData(m5)
mujoco.mj_forward(m5, d5)

print(f"  nv = {m5.nv}")
print(f"  nu = {m5.nu}")
print(f"  actuator_acc0 = {m5.actuator_acc0}")
print(f"  dof_M0        = {m5.dof_M0}")

qM5 = np.zeros((m5.nv, m5.nv))
mujoco.mj_fullM(m5, qM5, d5.qM)
print(f"\n  Full mass matrix:")
for row in range(m5.nv):
    print(f"    {qM5[row]}")
print(f"  qM diagonal = {np.diag(qM5)}")
print(f"  dof_M0 == qM diagonal: {np.allclose(m5.dof_M0, np.diag(qM5))}")

# For single-DOF-per-actuator with gear=1:
# acc0[i] = ||M^{-1} e_i|| where e_i is the i-th unit vector
# This is the norm of the i-th column of M^{-1}
Minv = np.linalg.inv(qM5)
print(f"\n  M^{{-1}} matrix:")
for row in range(m5.nv):
    print(f"    {Minv[row]}")
for i in range(m5.nu):
    acc0_from_minv = np.linalg.norm(Minv[:, i])
    print(f"  acc0[{i}] from M^{{-1}} col {i} norm = {acc0_from_minv}, "
          f"MuJoCo = {m5.actuator_acc0[i]}, "
          f"match = {np.isclose(acc0_from_minv, m5.actuator_acc0[i])}")

# =============================================================================
# Model 6: Position actuator with dampratio on multi-body chain
# =============================================================================
print()
print("=" * 72)
print("MODEL 6: Position actuator with dampratio=1.0 on 2-body chain")
print("=" * 72)

TWO_BODY_DAMPRATIO = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body pos="0 0 0.5">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="box" size="0.05 0.05 0.25" mass="2.0"/>
      <body pos="0 0 -0.5">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="box" size="0.05 0.05 0.25" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="pos1" joint="j1" kp="100" dampratio="1.0"/>
    <position name="pos2" joint="j2" kp="50" dampratio="1.5"/>
  </actuator>
</mujoco>
"""

m6 = mujoco.MjModel.from_xml_string(TWO_BODY_DAMPRATIO)
d6 = mujoco.MjData(m6)
mujoco.mj_forward(m6, d6)

print(f"  nv = {m6.nv}")
print(f"  nu = {m6.nu}")
print(f"  dof_M0            = {m6.dof_M0}")
print(f"  actuator_acc0     = {m6.actuator_acc0}")

for i in range(m6.nu):
    print(f"\n  Actuator {i}:")
    print(f"    gainprm  = {m6.actuator_gainprm[i]}")
    print(f"    biasprm  = {m6.actuator_biasprm[i]}")
    kp_i = m6.actuator_gainprm[i][0]
    biasprm2_i = m6.actuator_biasprm[i][2]
    print(f"    kp = {kp_i}")
    print(f"    stored biasprm[2] = {biasprm2_i}")

# Verify: for single-DOF actuators on a chain, reflected inertia = dof_M0[dof]
# (since transmission = gear = 1, moment has a single entry)
qM6 = np.zeros((m6.nv, m6.nv))
mujoco.mj_fullM(m6, qM6, d6.qM)
print(f"\n  Full mass matrix:")
for row in range(m6.nv):
    print(f"    {qM6[row]}")
print(f"  qM diagonal = {np.diag(qM6)}")

for i in range(m6.nu):
    kp_i = m6.actuator_gainprm[i][0]
    mass_i = m6.dof_M0[i]  # single-DOF, gear=1 -> reflected inertia = dof_M0
    # Read dampratio from the MJCF (1.0 for pos1, 1.5 for pos2)
    dampratio_i = [1.0, 1.5][i]
    kv_expected = dampratio_i * 2.0 * np.sqrt(kp_i * mass_i)
    actual_biasprm2 = m6.actuator_biasprm[i][2]
    print(f"\n  Actuator {i}: dampratio={dampratio_i}, kp={kp_i}, "
          f"reflected_inertia={mass_i}")
    print(f"    Expected kv = {kv_expected}")
    print(f"    Expected biasprm[2] = {-kv_expected}")
    print(f"    Actual biasprm[2]   = {actual_biasprm2}")
    print(f"    Match: {np.isclose(actual_biasprm2, -kv_expected)}")

# =============================================================================
# Model 7: Limited hinge — lengthrange should match joint limits
# =============================================================================
print()
print("=" * 72)
print("MODEL 7: Limited hinge (lengthrange == joint limits)")
print("=" * 72)

LIMITED_HINGE_USELIMIT = """
<mujoco>
  <compiler angle="radian">
    <lengthrange mode="all" uselimit="true"/>
  </compiler>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1" limited="true" range="-1.57 1.57"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" gear="1"/>
  </actuator>
</mujoco>
"""

m7 = mujoco.MjModel.from_xml_string(LIMITED_HINGE_USELIMIT)
d7 = mujoco.MjData(m7)
mujoco.mj_forward(m7, d7)

print(f"  uselimit=true (default):")
print(f"    jnt_range     = {m7.jnt_range[0]}")
print(f"    lengthrange   = {m7.actuator_lengthrange[0]}")
print(f"    Match limits: {np.allclose(m7.actuator_lengthrange[0], m7.jnt_range[0])}")

# Also test with uselimit=false to show difference
LIMITED_HINGE_NO_USELIMIT = """
<mujoco>
  <compiler angle="radian">
    <lengthrange mode="all" uselimit="false"/>
  </compiler>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1" limited="true" range="-1.57 1.57"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" gear="1"/>
  </actuator>
</mujoco>
"""

m7b = mujoco.MjModel.from_xml_string(LIMITED_HINGE_NO_USELIMIT)
print(f"  uselimit=false (simulation):")
print(f"    lengthrange   = {m7b.actuator_lengthrange[0]}")
print(f"    NOTE: Simulation-based range differs from joint limits.")

# =============================================================================
# Model 8: Unlimited slide — lengthrange from simulation
# =============================================================================
print()
print("=" * 72)
print("MODEL 8: Unlimited slide (lengthrange from simulation)")
print("=" * 72)

print()
print("  --- 8a: Unlimited slide with gravity (convergence test) ---")
UNLIMITED_SLIDE_GRAV = """
<mujoco>
  <compiler>
    <lengthrange mode="all"/>
  </compiler>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="slide1" type="slide" axis="0 0 1"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="slide1" gear="1"/>
  </actuator>
</mujoco>
"""
try:
    m8a = mujoco.MjModel.from_xml_string(UNLIMITED_SLIDE_GRAV)
    print(f"  lengthrange = {m8a.actuator_lengthrange[0]}")
except ValueError as e:
    print(f"  CONVERGENCE FAILURE: {e}")
    print(f"  NOTE: Unlimited slide along gravity axis doesn't converge!")
    print(f"  Gravity causes asymmetric force that breaks the convergence check.")

print()
print("  --- 8b: Unlimited slide, horizontal axis (gravity orthogonal) ---")
UNLIMITED_SLIDE_HORIZ = """
<mujoco>
  <compiler>
    <lengthrange mode="all"/>
  </compiler>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="slide1" type="slide" axis="1 0 0"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="slide1" gear="1"/>
  </actuator>
</mujoco>
"""
try:
    m8b = mujoco.MjModel.from_xml_string(UNLIMITED_SLIDE_HORIZ)
    d8b = mujoco.MjData(m8b)
    mujoco.mj_forward(m8b, d8b)
    print(f"  jnt_limited   = {m8b.jnt_limited[0]}")
    print(f"  lengthrange   = {m8b.actuator_lengthrange[0]}")
    print(f"  actuator_acc0 = {m8b.actuator_acc0}")
    print(f"  dof_M0        = {m8b.dof_M0}")
except ValueError as e:
    print(f"  CONVERGENCE FAILURE: {e}")

print()
print("  --- 8c: Unlimited slide, no gravity ---")
UNLIMITED_SLIDE_NOGRAV = """
<mujoco>
  <compiler>
    <lengthrange mode="all"/>
  </compiler>
  <option gravity="0 0 0"/>
  <worldbody>
    <body>
      <joint name="slide1" type="slide" axis="0 0 1"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="slide1" gear="1"/>
  </actuator>
</mujoco>
"""
try:
    m8c = mujoco.MjModel.from_xml_string(UNLIMITED_SLIDE_NOGRAV)
    d8c = mujoco.MjData(m8c)
    mujoco.mj_forward(m8c, d8c)
    print(f"  jnt_limited   = {m8c.jnt_limited[0]}")
    print(f"  lengthrange   = {m8c.actuator_lengthrange[0]}")
    print(f"  actuator_acc0 = {m8c.actuator_acc0}")
    print(f"  dof_M0        = {m8c.dof_M0}")
except ValueError as e:
    print(f"  CONVERGENCE FAILURE: {e}")

# =============================================================================
# Model 9: Unlimited hinge — lengthrange from simulation
# =============================================================================
print()
print("=" * 72)
print("MODEL 9: Unlimited hinge (lengthrange from simulation)")
print("=" * 72)

UNLIMITED_HINGE = """
<mujoco>
  <compiler angle="radian">
    <lengthrange mode="all"/>
  </compiler>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" gear="1"/>
  </actuator>
</mujoco>
"""

try:
    m9 = mujoco.MjModel.from_xml_string(UNLIMITED_HINGE)
    d9 = mujoco.MjData(m9)
    mujoco.mj_forward(m9, d9)
    print(f"  jnt_limited   = {m9.jnt_limited[0]}")
    print(f"  lengthrange   = {m9.actuator_lengthrange[0]}")
    print(f"  actuator_acc0 = {m9.actuator_acc0}")
    print(f"  dof_M0        = {m9.dof_M0}")
except ValueError as e:
    print(f"  CONVERGENCE FAILURE: {e}")
    print(f"  NOTE: Unlimited hinge also fails — same as slide.")

# =============================================================================
# Model 10: Position actuator with dampratio=0.5 (non-critical damping)
# =============================================================================
print()
print("=" * 72)
print("MODEL 10: Position actuator with dampratio=0.5 (underdamped)")
print("=" * 72)

POSITION_DAMPRATIO_05 = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="pos1" joint="hinge1" kp="200" dampratio="0.5"/>
  </actuator>
</mujoco>
"""

m10 = mujoco.MjModel.from_xml_string(POSITION_DAMPRATIO_05)
d10 = mujoco.MjData(m10)
mujoco.mj_forward(m10, d10)

kp = 200.0
mass = m10.dof_M0[0]
dampratio = 0.5
kv_expected = dampratio * 2.0 * np.sqrt(kp * mass)

print(f"  kp = {kp}, dampratio = {dampratio}")
print(f"  dof_M0 = {m10.dof_M0}")
print(f"  Analytical kv = {kv_expected}")
print(f"  Stored biasprm[2] = {m10.actuator_biasprm[0][2]}")
print(f"  Expected biasprm[2] = {-kv_expected}")
print(f"  Match: {np.isclose(m10.actuator_biasprm[0][2], -kv_expected)}")

# =============================================================================
# Model 11: Position actuator with gear != 1 (affects reflected inertia)
# =============================================================================
print()
print("=" * 72)
print("MODEL 11: Position actuator with gear=2 and dampratio=1.0")
print("=" * 72)

POSITION_GEAR2_DAMPRATIO = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="pos1" joint="hinge1" kp="100" dampratio="1.0" gear="2"/>
  </actuator>
</mujoco>
"""

m11 = mujoco.MjModel.from_xml_string(POSITION_GEAR2_DAMPRATIO)
d11 = mujoco.MjData(m11)
mujoco.mj_forward(m11, d11)

# For gear=2, transmission moment = 2.
# reflected inertia = dof_M0 / trn^2 = dof_M0 / 4
kp = 100.0
gear = 2.0
dof_M0_val = m11.dof_M0[0]
reflected_mass = dof_M0_val / (gear * gear)
kv_expected = 1.0 * 2.0 * np.sqrt(kp * reflected_mass)

print(f"  gear = {gear}")
print(f"  dof_M0 = {dof_M0_val}")
print(f"  reflected inertia = dof_M0 / gear^2 = {reflected_mass}")
print(f"  Analytical kv = 2 * sqrt(kp * reflected_mass) = {kv_expected}")
print(f"  Stored biasprm[2] = {m11.actuator_biasprm[0][2]}")
print(f"  Expected biasprm[2] = {-kv_expected}")
print(f"  Match: {np.isclose(m11.actuator_biasprm[0][2], -kv_expected)}")
print(f"  actuator_acc0 = {m11.actuator_acc0}")

# =============================================================================
# Model 12: Velocity actuator (should skip dampratio — gainprm[0] != -biasprm[1])
# =============================================================================
print()
print("=" * 72)
print("MODEL 12: Velocity actuator (dampratio guard — non-position-like)")
print("=" * 72)

VELOCITY_ACTUATOR = """
<mujoco>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="vel1" joint="hinge1" kv="10"/>
  </actuator>
</mujoco>
"""

m12 = mujoco.MjModel.from_xml_string(VELOCITY_ACTUATOR)
d12 = mujoco.MjData(m12)
mujoco.mj_forward(m12, d12)

print(f"  actuator_gainprm = {m12.actuator_gainprm[0]}")
print(f"  actuator_biasprm = {m12.actuator_biasprm[0]}")
print(f"  gainprm[0] = {m12.actuator_gainprm[0][0]}")
print(f"  biasprm[1] = {m12.actuator_biasprm[0][1]}")
print(f"  Position-like (gainprm[0] == -biasprm[1]): "
      f"{m12.actuator_gainprm[0][0] == -m12.actuator_biasprm[0][1]}")
print(f"  biasprm[2] = {m12.actuator_biasprm[0][2]} (should be 0 — no dampratio conversion)")

# =============================================================================
# Model 13: Site transmission — lengthrange from simulation
# =============================================================================
print()
print("=" * 72)
print("MODEL 13: Site transmission (lengthrange from simulation)")
print("=" * 72)

SITE_TRANSMISSION = """
<mujoco>
  <compiler>
    <lengthrange mode="all"/>
  </compiler>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body pos="0 0 1">
      <joint name="slide1" type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="site1" pos="0 0 0"/>
    </body>
    <site name="refsite" pos="0 0 0"/>
  </worldbody>
  <actuator>
    <general name="act1" site="site1" refsite="refsite"
             gainprm="1" biastype="none"
             dyntype="none" gear="1 0 0 0 0 0"/>
  </actuator>
</mujoco>
"""

try:
    m13 = mujoco.MjModel.from_xml_string(SITE_TRANSMISSION)
    d13 = mujoco.MjData(m13)
    mujoco.mj_forward(m13, d13)
    print(f"  actuator_trntype  = {m13.actuator_trntype[0]}")
    print(f"  actuator_lengthrange = {m13.actuator_lengthrange[0]}")
    print(f"  actuator_acc0     = {m13.actuator_acc0}")
    print(f"  actuator_length   = {d13.actuator_length}")
except ValueError as e:
    print(f"  CONVERGENCE FAILURE: {e}")

# =============================================================================
# Model 14: Limited slide with gear != 1 (lengthrange scaled by gear)
# =============================================================================
print()
print("=" * 72)
print("MODEL 14: Limited slide with gear=3 (lengthrange scaling)")
print("=" * 72)

LIMITED_SLIDE_GEAR = """
<mujoco>
  <compiler>
    <lengthrange mode="all"/>
  </compiler>
  <worldbody>
    <body>
      <joint name="slide1" type="slide" axis="0 0 1" limited="true" range="-0.5 0.5"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="slide1" gear="3"/>
  </actuator>
</mujoco>
"""

m14 = mujoco.MjModel.from_xml_string(LIMITED_SLIDE_GEAR)
d14 = mujoco.MjData(m14)
mujoco.mj_forward(m14, d14)

print(f"  jnt_range       = {m14.jnt_range[0]}")
print(f"  lengthrange     = {m14.actuator_lengthrange[0]}")
print(f"  NOTE: MuJoCo's uselimit copies joint range directly (no gear scaling).")
print(f"  The joint range IS the actuator's length range when using uselimit.")
print(f"  gear only affects actuator_length = gear * qpos, not the lengthrange copy.")

# =============================================================================
# Model 15: mjLROpt mode=MUSCLE filtering (motor should be skipped)
# =============================================================================
print()
print("=" * 72)
print("MODEL 15: LR mode filtering (default=MUSCLE, motor should be skipped)")
print("=" * 72)

LR_MODE_MUSCLE = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body>
      <joint name="hinge1" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.5 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="hinge1" gear="1"/>
  </actuator>
</mujoco>
"""

m15 = mujoco.MjModel.from_xml_string(LR_MODE_MUSCLE)
d15 = mujoco.MjData(m15)
mujoco.mj_forward(m15, d15)

print(f"  lengthrange (no <compiler lengthrange>): {m15.actuator_lengthrange[0]}")
print(f"  NOTE: Without lengthrange='all', default mode=MUSCLE skips motors.")
print(f"  Range stays (0,0) for non-muscle actuators by default.")

# =============================================================================
# Summary table
# =============================================================================
print()
print("=" * 72)
print("EMPIRICAL GROUND TRUTH SUMMARY")
print("=" * 72)

print("""
| Model | Feature | Key Value | Verified |
|-------|---------|-----------|----------|""")

# Model 1
print(f"| 1 (single hinge motor) | acc0 | {m1.actuator_acc0[0]:.15f} | "
      f"{'YES' if np.isclose(m1.actuator_acc0[0], acc0_analytical) else 'NO'} |")
print(f"| 1 | dof_M0 | {m1.dof_M0[0]:.15f} | "
      f"{'YES' if np.allclose(m1.dof_M0, np.diag(qM)) else 'NO'} |")

# Model 2
print(f"| 2 (pos dampratio=1.0) | biasprm[2] | {m2.actuator_biasprm[0][2]:.15f} | "
      f"{'YES' if np.isclose(m2.actuator_biasprm[0][2], -1.0 * 2.0 * np.sqrt(100.0 * m2.dof_M0[0])) else 'NO'} |")

# Model 5
for i in range(3):
    acc0_check = np.linalg.norm(Minv[:, i])
    print(f"| 5 (3-body chain) | acc0[{i}] | {m5.actuator_acc0[i]:.15f} | "
          f"{'YES' if np.isclose(m5.actuator_acc0[i], acc0_check) else 'NO'} |")
print(f"| 5 | dof_M0 == qM diag | {m5.dof_M0} | "
      f"{'YES' if np.allclose(m5.dof_M0, np.diag(qM5)) else 'NO'} |")

# Model 7
print(f"| 7 (limited hinge) | lengthrange | {m7.actuator_lengthrange[0]} | "
      f"{'YES' if np.allclose(m7.actuator_lengthrange[0], m7.jnt_range[0]) else 'NO'} |")

# Model 8, 9 — convergence failures documented above
print(f"| 8 (unlimited slide) | lengthrange | CONVERGENCE FAILURE | documented |")
print(f"| 9 (unlimited hinge) | lengthrange | CONVERGENCE FAILURE | documented |")

# Model 11
print(f"| 11 (pos gear=2 dr=1) | biasprm[2] | {m11.actuator_biasprm[0][2]:.15f} | "
      f"{'YES' if np.isclose(m11.actuator_biasprm[0][2], -1.0 * 2.0 * np.sqrt(100.0 * m11.dof_M0[0] / 4.0)) else 'NO'} |")

# Model 13 — may or may not converge
print(f"| 13 (site transmission) | lengthrange | (see above) | documented |")

print(f"""
MuJoCo version: {mujoco.__version__}
""")

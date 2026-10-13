#!/usr/bin/env python3
"""Print MuJoCo 3.5.0's touch readings that `sensor_phase6_spec_c.rs` holds as literals.

Run it with the unfused oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_touch_cutoff_reference.py

It refuses any other interpreter, as gen_census_golden.py does.

A free sphere resting on a plane with three touch sensors on one site: one
with a cutoff of 1, one with a cutoff of 1 delayed two steps (nsample 3),
and one with no cutoff. It prints the three readings after steps 1, 2, 3, 4,
5, 10, 50 and 100 (MuJoCo clamps a touch sensor's reading at its cutoff,
`engine_sensor.c:63-88`; ledger L73).
"""
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402

XML = """<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
      <site name="s" type="sphere" size="0.12"/>
    </body>
  </worldbody>
  <sensor>
    <touch name="clamped" site="s" cutoff="1"/>
    <touch name="delayed" site="s" cutoff="1" nsample="3" delay="0.004"/>
    <touch name="raw" site="s"/>
  </sensor>
</mujoco>"""


def main():
    oracle()
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    print("step", "clamped", "delayed", "no cutoff")
    for step in range(1, 101):
        mujoco.mj_step(m, d)
        if step in (1, 2, 3, 4, 5, 10, 50, 100):
            print(step, *(repr(float(x)) for x in d.sensordata))


if __name__ == "__main__":
    main()

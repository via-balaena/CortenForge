#!/usr/bin/env python3
"""Generate sim-mjcf's built-in mesh golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_mesh_builtin_reference.py \\
        sim/L0/mjcf/tests/assets/mesh_builtin/mjs_makemesh.txt

It refuses any other interpreter, as gen_census_golden.py does.

Each case is one `<mesh builtin params>`, read by MjSpec.from_string, whose
reader has mjs_makeMesh make the mesh; it is written beside a face "0 1 2"
and inertia "exact", so that the golden shows whether the generator replaced
them. `parser/mesh_builtin.rs`'s test makes each case's mesh with the same
parameters and compares it with the file's (the file's header says how).
"""
import os
import struct
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402

HEADER = """# MuJoCo 3.5.0 (built from its 3.5.0 tag with -ffp-contract=off, macOS arm64) reading
# <mesh name="m" builtin="KIND" params="PARAMS" face="0 1 2" inertia="exact"/> with MjSpec.from_string, which
# has mjs_makeMesh generate the mesh: `case KIND PARAMS`, then `err` MuJoCo's refusal, or `v` every vertex
# coordinate's float bits, `f` the face ids after it (`0 1 2` where the generator writes none), `inertia` the
# mode after it; `vfnv`/`ffnv` in place of `v`/`f` are FNV-1a 64 hashes of the floats' or the ids' little-endian
# bytes and their count (a vertex hash of a generator that calls cos, sin or pow is compared on macOS arm64 only;
# an id hash for a hashed case and for more than 1,000 ids).
# Written by sim/L0/tests/scripts/gen_mesh_builtin_reference.py; the oracle's libraries' sha256 (without UUID):
# {oracle}
# `parser/mesh_builtin.rs`'s test makes each case's mesh with the same parameters."""

# (kind, params) or (kind, params, 'hash'): a hash of the vertices, and of the faces where there are faces.
CASES = [
    # sphere: subdivisions 0..4; the int cast; each check
    ('sphere', '0'), ('sphere', '1'), ('sphere', '2'), ('sphere', '3'), ('sphere', '4', 'hash'),
    ('sphere', '1.7'), ('sphere', '-0.5'), ('sphere', '4.9', 'hash'),
    ('sphere', '5'), ('sphere', '-1'), ('sphere', '1e10'), ('sphere', '-1e10'), ('sphere', '2 3'),
    # hemisphere: 0..10
    ('hemisphere', '0'), ('hemisphere', '1'), ('hemisphere', '2'), ('hemisphere', '3'), ('hemisphere', '10', 'hash'),
    ('hemisphere', '11'), ('hemisphere', '-1'), ('hemisphere', '1 1'),
    # supersphere: res >= 3, e >= 0, n >= 0
    ('supersphere', '3 1 1'), ('supersphere', '4 1 1'), ('supersphere', '8 0 1'), ('supersphere', '8 1 0'),
    ('supersphere', '8 0 0'), ('supersphere', '7 0.5 3'), ('supersphere', '12 1.5 0.7'), ('supersphere', '13 0.4 2'),
    ('supersphere', '16 2 0.4'), ('supersphere', '30 0.2 0.3'),
    ('supersphere', '2 1 1'), ('supersphere', '2.9 1 1'), ('supersphere', '8 -0.1 1'), ('supersphere', '8 1 -0.1'),
    ('supersphere', '8 1'),
    # supertorus: res >= 3, radius in (0, 1], s > 0, t > 0
    ('supertorus', '3 0.5 1 1'), ('supertorus', '8 1 1 1'), ('supertorus', '12 0.25 0.3 3'),
    ('supertorus', '9 0.7 2 0.5'), ('supertorus', '16 0.5 0.4 0.4'), ('supertorus', '30 0.5 2 2'),
    ('supertorus', '2 0.5 1 1'), ('supertorus', '8 0 1 1'), ('supertorus', '8 1.01 1 1'), ('supertorus', '8 -0.5 1 1'),
    ('supertorus', '8 0.5 0 1'), ('supertorus', '8 0.5 1 0'), ('supertorus', '8 0.5 1'),
    # wedge: fov[0] in (0, 180], fov[1] in (0, 90], resolutions > 0, gamma in [0, 1]
    ('wedge', '1 1 90 45 0'), ('wedge', '2 2 90 45 0.25'), ('wedge', '4 4 60 45 0.5'), ('wedge', '5 3 120 30 1'),
    ('wedge', '9 7 180 90 0.3'), ('wedge', '12 10 45 20 0'),
    ('wedge', '4 4 0 45 0'), ('wedge', '4 4 181 45 0'), ('wedge', '4 4 90 0 0'), ('wedge', '4 4 90 91 0'),
    ('wedge', '0 4 90 45 0'), ('wedge', '4 -2 90 45 0'), ('wedge', '4 4 90 45 -0.1'), ('wedge', '4 4 90 45 1.1'),
    ('wedge', '4 4 90 45'),
    # plate: resolutions > 0; a shell
    ('plate', '1 1'), ('plate', '1 4'), ('plate', '2 2'), ('plate', '2 3'), ('plate', '3 5'), ('plate', '6 2'),
    ('plate', '37 37', 'hash'),
    ('plate', '0 2'), ('plate', '2 -1'), ('plate', '2'),
    # cone: no checks but the count
    ('cone', '0 0.5'), ('cone', '0 0'), ('cone', '1 0.5'), ('cone', '2 0.5'), ('cone', '3 1'), ('cone', '5 0.5'),
    ('cone', '7 0'), ('cone', '8 0.5'), ('cone', '4 2'), ('cone', '5 -1'), ('cone', '3.9 0.5'),
    ('cone', '5'),
    # none
    ('none', '1'),
    # every hemisphere resolution the cases above leave out
    ('hemisphere', '4', 'hash'), ('hemisphere', '5', 'hash'), ('hemisphere', '6', 'hash'),
    ('hemisphere', '7', 'hash'), ('hemisphere', '8', 'hash'), ('hemisphere', '9', 'hash'),
    # supertori of radii that are not powers of two
    ('supertorus', '16 0.3 1 1'), ('supertorus', '20 0.7 0.5 1.5'), ('supertorus', '25 0.9 3 0.3'),
    ('supertorus', '12 0.1 1 1'), ('supertorus', '7 0.35 1.7 0.8'),
    # more wedges, cones, superspheres
    ('wedge', '10 10 37 23 0'), ('wedge', '16 8 170 80 0.7'), ('wedge', '7 13 100 60 0.1'), ('wedge', '20 20 15 5 0'),
    ('cone', '6 0.5'), ('cone', '9 0.3'), ('cone', '12 0.7'), ('cone', '17 1.3'), ('cone', '25 0'), ('cone', '33 0.2'),
    ('supersphere', '17 0.7 1.3'), ('supersphere', '23 1.1 0.9'),
    # two checks failing at once: the one MuJoCo names first
    ('wedge', '0 4 0 45 0'), ('wedge', '4 4 0 0 0'), ('wedge', '0 0 90 45 2'), ('wedge', '4 4 90 91 2'),
    ('supersphere', '2 -1 1'), ('supersphere', '8 -1 -1'), ('supertorus', '2 0 1 1'), ('supertorus', '8 2 0 0'),
    ('supertorus', '8 0.5 0 0'),
    # an angle's sum, product or order of operations that a rounding to float shows
    ('cone', '22 0.5'), ('cone', '26 0'), ('wedge', '3 3 89 20 0'), ('wedge', '3 3 157 20 0'),
    # angles at which __sincos_stret's sine and sin's round a vertex to different floats (review round 1's
    # searches): each case fails when its angle's sine is taken from the other call than MuJoCo's build takes,
    # __sincos_stret for the wedge's, the supertorus's and the supersphere's v, sin for the supersphere's u
    ('wedge', '2 1 49.77219617845156 45.0 0.0'), ('wedge', '2 1 76.93697507539862 45.0 0.0'),
    ('wedge', '2 1 79.4821470198428 45.0 0.0'), ('wedge', '1 2 90.0 84.97358776835924 0.0'),
    ('wedge', '1 2 90.0 87.11669442434288 0.0'), ('wedge', '1 2 90.0 74.46280348425043 0.0'),
    ('wedge', '3 2 90.0 84.97358776835924 0.0'),
    ('supersphere', '40 1.5246355854531397 1.0', 'hash'), ('supersphere', '40 1.5246355854531397 0', 'hash'),
    ('supertorus', '29 1.0 1e300 2.39721118128265', 'hash'),
    ('supertorus', '37 1.0 1e300 1.6766900027980545', 'hash'),
    ('supertorus', '40 1.0 1e300 1.5246355854531397', 'hash'),
    ('supertorus', '58 1.0 1e300 2.39721118128265', 'hash'),
    ('supertorus', '65 1.0 1e300 2.0017596890764535', 'hash'),
    ('supersphere', '19 1.0 3.0897383245849674', 'hash'), ('supersphere', '20 1.0 1.5246355854531397', 'hash'),
    ('supersphere', '25 1.0 1.0252169133610043', 'hash'),
    ('supertorus', '29 0.5 2.39721118128265 1.0', 'hash'), ('supertorus', '37 0.5 1.6766900027980545 1.0', 'hash'),
]

INERTIA = {0: 'convex', 1: 'exact', 2: 'legacy', 3: 'shell'}


def fnv(data):
    h = 0xcbf29ce484222325
    for b in data:
        h = ((h ^ b) * 0x100000001b3) & 0xffffffffffffffff
    return '%016x' % h


def case_lines(kind, params, hashed):
    xml = (f'<mujoco><asset><mesh name="m" builtin="{kind}" params="{params}" face="0 1 2" inertia="exact"/>'
           '</asset></mujoco>')
    out = [f'case {kind} {params}']
    try:
        spec = mujoco.MjSpec.from_string(xml)
    except Exception as e:  # MuJoCo's refusal, in its words
        msg = str(e).split('\n')[0]
        for prefix in ('XML Error: ', 'Error: '):
            if msg.startswith(prefix):
                msg = msg[len(prefix):]
        return out + [f'err {msg}']
    m = spec.meshes[0]
    vert = [float(x) for x in m.uservert]
    face = [int(x) for x in m.userface]
    if hashed:
        out.append('vfnv ' + fnv(b''.join(struct.pack('<f', x) for x in vert)) + f' {len(vert)}')
    else:
        out.append('v ' + ' '.join(struct.pack('>f', x).hex() for x in vert))
    if hashed or len(face) > 1000:
        out.append('ffnv ' + fnv(b''.join(struct.pack('<I', i) for i in face)) + f' {len(face)}')
    else:
        out.append('f ' + ' '.join(str(i) for i in face))
    out.append(f'inertia {INERTIA[int(m.inertia)]}')
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    out = [HEADER.format(oracle=marker['libraries_sha256_without_uuid'])]
    for case in CASES:
        out += case_lines(case[0], case[1], len(case) > 2)
    with open(sys.argv[1], 'w', newline='\n') as f:
        f.write('\n'.join(out) + '\n')
    print(f'{len(CASES)} cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()

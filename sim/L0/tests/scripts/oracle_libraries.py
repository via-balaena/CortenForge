#!/usr/bin/env python3
"""The hash that names an oracle build, as build_mujoco_oracle.sh writes it.

    python3 -I oracle_libraries.py <the mujoco package directory>

prints sha256 over "<path in the package>\\t<sha256>\\n", one line per library
(*.dylib, *.so, *.so.*; links skipped), in byte order of the path, where each
library's sha256 is over its bytes without its LC_UUID and its code signature
(64-bit Mach-O), or over all its bytes (ELF).
build_mujoco_oracle.sh records it in oracle.json as
libraries_sha256_without_uuid, and gen_census_golden.py refuses an interpreter
whose mujoco package does not hash to it.

The LC_UUID is left out because the linker gave one binding module
(_specs.cpython-312-darwin.so) a different UUID in one of six builds, whose
bytes were otherwise identical once the UUID was zeroed and the code
signature cut (2026-10-21; what made it differ is not isolated). The ad-hoc
code signature goes with it, since it hashes the page holding the UUID.
Neither is executed. The build directory itself is kept out of the libraries
by the build (CMAKE_SKIP_BUILD_RPATH, and a check that refuses a library
naming it; see build_mujoco_oracle.sh).

The goldens are blessed on Darwin-arm64. An ELF library (Linux) is hashed
whole, as every library was before the UUID was left out; no two Linux
builds have been compared.
"""
import fnmatch
import hashlib
import os
import struct
import sys

MH_MAGIC_64 = 0xFEEDFACF
ELF_MAGIC = b'\x7fELF'
LC_UUID = 0x1B
LC_CODE_SIGNATURE = 0x1D
LIBRARIES = ('*.dylib', '*.so', '*.so.*')


def library_sha256(data, name):
    """sha256 of one library's bytes: a 64-bit Mach-O's with its LC_UUID zeroed
    and its code signature cut, an ELF's whole."""
    if data[:4] == ELF_MAGIC:
        return hashlib.sha256(data).hexdigest()
    if len(data) < 32 or struct.unpack_from('<I', data, 0)[0] != MH_MAGIC_64:
        sys.exit(f'{name}: neither a 64-bit Mach-O nor an ELF library')
    data = bytearray(data)
    ncmds = struct.unpack_from('<I', data, 16)[0]
    signature = None
    off = 32
    for _ in range(ncmds):
        cmd, size = struct.unpack_from('<2I', data, off)
        if cmd == LC_UUID:
            data[off + 8:off + 24] = bytes(16)
        elif cmd == LC_CODE_SIGNATURE:
            dataoff, datasize = struct.unpack_from('<2I', data, off + 8)
            signature = (dataoff, dataoff + datasize)
        off += size
    if signature is not None:
        del data[signature[0]:signature[1]]
    return hashlib.sha256(data).hexdigest()


def libraries_sha256(pkg):
    """The oracle's hash over every library under the package directory `pkg`."""
    lines = []
    for root, _, files in os.walk(pkg):
        for name in files:
            path = os.path.join(root, name)
            if os.path.islink(path) or not any(fnmatch.fnmatch(name, g) for g in LIBRARIES):
                continue
            with open(path, 'rb') as f:
                lines.append((os.path.relpath(path, pkg), library_sha256(f.read(), path)))
    if not lines:
        sys.exit(f'{pkg}: no libraries')
    text = ''.join(f'{rel}\t{digest}\n' for rel, digest in sorted(lines))
    return hashlib.sha256(text.encode()).hexdigest()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    print(libraries_sha256(sys.argv[1]))

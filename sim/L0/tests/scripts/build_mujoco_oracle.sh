#!/usr/bin/env bash
# Build the unfused MuJoCo 3.5.0 oracle: the C library, its plugins and its
# Python bindings, compiled from the 3.5.0 source tag with -ffp-contract=off.
#
# The census goldens (gen_census_golden.py) and every bit-exact golden in the
# Rigid series come from this build, not from the PyPI wheel: the arm64 wheel
# fuses multiply-adds and sim-core's arithmetic does not. See "The unfused
# oracle" in docs/studies/a_double_dose_of_detail/src/40-verification.md.
#
# Usage: build_mujoco_oracle.sh <workdir>
#   <workdir> is created if missing and must be empty. Keep it outside the
#   repository. The oracle is <workdir>/venv/bin/python, which imports it as
#   `mujoco`.
#
# Needs git, cmake, a C/C++ compiler, objdump, uv, and network access: MuJoCo's
# CMake fetches its dependencies at pinned commits. The steps are MuJoCo's own CI
# steps (.github/workflows/build_steps.sh at the tag) with the flag added.
set -euo pipefail

readonly TAG=3.5.0
readonly COMMIT=881544c0c58dc2e95fbd132a5ec90b99e012b6f7
readonly FLAGS=-ffp-contract=off
readonly PYTHON=3.12
readonly JOBS=${JOBS:-8}

die() { echo "build_mujoco_oracle: $*" >&2; exit 1; }

[[ $# -eq 1 ]] || die "usage: build_mujoco_oracle.sh <workdir>"
mkdir -p "$1"
[[ -z "$(ls -A "$1")" ]] || die "$1 is not empty"
work="$(cd "$1" && pwd)"
src="$work/src"
install="$work/install"
plugins="$install/mujoco_plugin"
py="$work/venv/bin/python"

echo "== source: MuJoCo $TAG"
git clone --quiet --depth 1 --branch "$TAG" https://github.com/google-deepmind/mujoco.git "$src"
[[ "$(git -C "$src" rev-parse HEAD)" == "$COMMIT" ]] || die "tag $TAG is not $COMMIT"

echo "== C library and plugins"
cmake -S "$src" -B "$work/build" \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
    -DCMAKE_INSTALL_PREFIX:STRING="$install" \
    -DCMAKE_C_FLAGS:STRING="$FLAGS" \
    -DCMAKE_CXX_FLAGS:STRING="$FLAGS" \
    -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
    -DMUJOCO_BUILD_SIMULATE:BOOL=OFF \
    -DMUJOCO_BUILD_TESTS:BOOL=OFF \
    -DMUJOCO_TEST_PYTHON_UTIL:BOOL=OFF
cmake --build "$work/build" --config Release -j "$JOBS"
cmake --install "$work/build"
mkdir -p "$plugins"
for p in actuator elasticity sensor sdf_plugin; do
    cp "$work/build/lib/lib$p".* "$plugins/"
done

echo "== Python environment"
uv venv --quiet --python "$PYTHON" "$work/venv"
uv pip install --quiet --python "$py" --require-hashes \
    -r "$src/python/build_requirements.txt"

echo "== Python sdist (the steps of python/make_sdist.sh)"
sdist="$work/sdist"
mkdir -p "$sdist"
cp -r "$src/python/." "$sdist"
codegen="$src/python/mujoco/codegen"
PYTHONPATH="$src/python/mujoco/python/.." "$py" "$codegen/generate_enum_traits.py" \
    > "$sdist/mujoco/enum_traits.h"
PYTHONPATH="$src/python/mujoco/python/.." "$py" "$codegen/generate_function_traits.py" \
    > "$sdist/mujoco/function_traits.h"
PYTHONPATH="$src/python/mujoco/python/.." "$py" "$codegen/generate_spec_bindings.py" \
    > "$sdist/mujoco/specs.cc.inc"
cp "$src/LICENSE" "$sdist"
mkdir -p "$sdist/mujoco/cmake"
cp "$src"/cmake/*.cmake "$sdist/mujoco/cmake"
cp -r "$src/simulate" "$sdist/mujoco"
(cd "$sdist" && "$py" -m build . --sdist --outdir "$work/dist")

echo "== Python bindings"
MUJOCO_PATH="$install" \
MUJOCO_PLUGIN_PATH="$plugins" \
MUJOCO_CMAKE_ARGS="-DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DCMAKE_C_FLAGS:STRING=$FLAGS -DCMAKE_CXX_FLAGS:STRING=$FLAGS" \
    "$py" -m pip wheel --quiet --no-deps --wheel-dir "$work/dist" "$work/dist/mujoco-$TAG.tar.gz"
uv pip install --quiet --python "$py" --no-deps "$work/dist/mujoco-$TAG"-*.whl

echo "== check: the venv imports this build"
(cd "$work" && "$py" -I -c '
import sys, mujoco
assert mujoco.mj_versionString() == sys.argv[1], mujoco.mj_versionString()
assert mujoco.__file__.startswith(sys.argv[2]), mujoco.__file__
print("mujoco", mujoco.mj_versionString(), "from", mujoco.__file__)
' "$TAG" "$work/venv/")

echo "== check: no fused multiply-add in any built binary"
case "$(uname -m)" in
    arm64|aarch64) fused='\b(fn?m(add|sub)|fml[as])\b' ;;
    x86_64) fused='\bvfn?m(add|sub)' ;;
    *) die "no fused-instruction pattern for $(uname -m)" ;;
esac
pkg="$("$py" -I -c 'import mujoco, os; print(os.path.dirname(mujoco.__file__))')"
total=0
while IFS= read -r bin; do
    n="$(objdump -d --no-show-raw-insn "$bin" | grep -cE "$fused" || true)"
    echo "$n $bin"
    total=$((total + n))
done < <(find "$install" "$pkg" -type f \( -name '*.dylib' -o -name '*.so' -o -name '*.so.*' \) | sort)
[[ "$total" -eq 0 ]] || die "$total fused multiply-add instructions found"

# gen_census_golden.py refuses any interpreter without this marker beside it.
cat > "$work/oracle.json" <<EOF
{"mujoco": "$TAG", "commit": "$COMMIT", "flags": "$FLAGS", "platform": "$(uname -s)-$(uname -m)"}
EOF
echo "oracle ready: $py"

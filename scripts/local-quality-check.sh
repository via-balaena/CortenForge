#!/bin/bash
# Local Quality Check - Mirrors the GitHub Actions quality-gate.yml
#
# Usage:
#   ./scripts/local-quality-check.sh        # Run all checks
#   ./scripts/local-quality-check.sh quick  # Format + Clippy + Tests only
#   ./scripts/local-quality-check.sh fmt    # Format only
#   ./scripts/local-quality-check.sh clippy # Clippy only
#   ./scripts/local-quality-check.sh test   # Tests only
#   ./scripts/local-quality-check.sh docs   # Docs only
#   ./scripts/local-quality-check.sh safety # Safety checks only
#   ./scripts/local-quality-check.sh deps   # Dependency checks only
#   ./scripts/local-quality-check.sh wasm   # WASM compatibility only
#   ./scripts/local-quality-check.sh bevy   # Bevy-free check only

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Match CI environment
export CARGO_TERM_COLOR=always
export RUSTFLAGS="-D warnings"
export RUSTDOCFLAGS="-D warnings"

print_header() {
    echo ""
    echo -e "${BLUE}══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}══════════════════════════════════════════════════════════════${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

check_format() {
    print_header "FORMAT CHECK"
    if cargo fmt --all -- --check; then
        print_success "Formatting is correct"
    else
        print_error "Formatting issues found. Run 'cargo fmt --all' to fix."
        return 1
    fi
}

check_clippy() {
    print_header "CLIPPY"
    if cargo clippy --all-targets --all-features -- -D warnings; then
        print_success "No Clippy warnings"
    else
        print_error "Clippy found issues"
        return 1
    fi
}

check_tests() {
    print_header "TESTS"
    if cargo test --all-features; then
        print_success "All tests passed"
    else
        print_error "Tests failed"
        return 1
    fi
}

check_docs() {
    print_header "DOCUMENTATION"
    if cargo doc --no-deps --all-features; then
        print_success "Documentation builds without warnings"
    else
        print_error "Documentation has warnings or errors"
        return 1
    fi
}

check_safety() {
    print_header "SAFETY CHECKS"

    # Check for unwrap/expect in library code
    FOUND_VIOLATIONS=false

    while IFS= read -r -d '' file; do
        VIOLATIONS=$(awk '
            /^[ \t]*#\[cfg\(test\)\]/ { in_test = 1 }
            !in_test && !/^[ \t]*\/\/[\/!]/ && /\.(unwrap|expect)\(/ { print NR": "$0 }
        ' "$file" 2>/dev/null)

        if [ -n "$VIOLATIONS" ]; then
            if [ "$FOUND_VIOLATIONS" = false ]; then
                print_error "Found unwrap/expect in library code:"
                FOUND_VIOLATIONS=true
            fi
            echo "  In $file:"
            echo "$VIOLATIONS" | head -5 | sed 's/^/    /'
        fi
    done < <(find . -name "*.rs" \
        -not -path "./target/*" \
        -not -path "./xtask/*" \
        -not -path "*/tests/*" \
        -not -path "*/examples/*" \
        -not -path "*/benches/*" \
        -print0 2>/dev/null)

    if [ "$FOUND_VIOLATIONS" = true ]; then
        return 1
    fi

    # Check for panic!/todo!/unimplemented!
    PANIC_FILES=$(find . -name "*.rs" \
        -not -path "./target/*" \
        -not -path "./xtask/*" \
        -not -path "*/tests/*" \
        -not -path "*/examples/*" \
        -exec grep -l -E 'panic!\(|todo!\(|unimplemented!\(' {} \; 2>/dev/null || true)

    if [ -n "$PANIC_FILES" ]; then
        print_warning "Found panic!/todo!/unimplemented! (review may be required):"
        echo "$PANIC_FILES" | sed 's/^/    /'
    fi

    print_success "No unwrap/expect found in library code"
}

check_security() {
    print_header "SECURITY AUDIT"

    if ! command -v cargo-audit &> /dev/null; then
        print_warning "cargo-audit not installed. Install with: cargo install cargo-audit"
        return 0
    fi

    if cargo audit --deny warnings; then
        print_success "No security vulnerabilities found"
    else
        print_warning "Security vulnerabilities found (see above)"
    fi
}

check_dependencies() {
    print_header "DEPENDENCY CHECK"

    if ! command -v cargo-deny &> /dev/null; then
        print_warning "cargo-deny not installed. Install with: cargo install cargo-deny"
        return 0
    fi

    if cargo deny check; then
        print_success "Dependencies pass policy checks"
    else
        print_error "Dependency policy violations found"
        return 1
    fi
}

check_wasm() {
    print_header "WASM COMPATIBILITY (Layer 0)"

    # Check if wasm target is installed
    if ! rustup target list --installed | grep -q wasm32-unknown-unknown; then
        print_warning "WASM target not installed. Install with: rustup target add wasm32-unknown-unknown"
        return 0
    fi

    WASM_CRATES=(
        "mesh-types"
        "mesh-geodesic"
        "mesh-transform"
        "mesh-measure"
        "mesh-slice"
        "sensor-types"
        "ml-types"
        "sim-types"
        "sim-core"
        "sim-contact"
        "sim-constraint"
        "sim-urdf"
        "sim-physics"
    )

    FAILED=false
    for crate in "${WASM_CRATES[@]}"; do
        if cargo check -p "$crate" --target wasm32-unknown-unknown 2>/dev/null; then
            print_success "$crate builds for WASM"
        else
            print_warning "$crate does not build for WASM (may need feature flags)"
        fi
    done
}

check_bevy_free() {
    print_header "BEVY-FREE CHECK (Layer 0)"

    LAYER0_CRATES=(
        "mesh-types"
        "mesh-io"
        "mesh-repair"
        "mesh-geodesic"
        "mesh-transform"
        "mesh-sdf"
        "mesh-offset"
        "mesh-zones"
        "mesh-from-curves"
        "mesh-shell"
        "mesh-measure"
        "mesh-thickness"
        "mesh-slice"
        "mesh-decimate"
        "mesh-subdivide"
        "mesh-remesh"
        "mesh-region"
        "mesh-assembly"
        "mesh-printability"
        "mesh-boolean"
        "mesh-registration"
        "mesh-morph"
        "mesh-scan"
        "mesh-lattice"
        "mesh-template"
        "sensor-types"
        "sensor-fusion"
        "ml-types"
        "ml-models"
        "ml-dataset"
        "ml-training"
        "sim-types"
        "sim-core"
        "sim-contact"
        "sim-constraint"
        "sim-urdf"
        "sim-physics"
    )

    FAILED=false
    for crate in "${LAYER0_CRATES[@]}"; do
        if cargo tree -p "$crate" 2>/dev/null | grep -qi "bevy"; then
            print_error "$crate has Bevy in dependency tree"
            FAILED=true
        fi
    done

    if [ "$FAILED" = true ]; then
        return 1
    fi
    print_success "All Layer 0 crates are Bevy-free"
}

check_coverage() {
    print_header "COVERAGE CHECK"

    if ! command -v cargo-tarpaulin &> /dev/null; then
        print_warning "cargo-tarpaulin not installed. Install with: cargo install cargo-tarpaulin"
        return 0
    fi

    if cargo tarpaulin --all-features --workspace --fail-under 75; then
        print_success "Coverage meets threshold (≥75%)"
    else
        print_error "Coverage below threshold"
        return 1
    fi
}

run_quick() {
    check_format
    check_clippy
    check_tests
}

run_all() {
    check_format
    check_clippy
    check_tests
    check_docs
    check_safety
    check_security
    check_dependencies
    check_bevy_free
    # Skipping WASM and coverage by default as they're slower
    # Run them explicitly if needed
}

# Main
case "${1:-all}" in
    quick)
        run_quick
        ;;
    fmt|format)
        check_format
        ;;
    clippy)
        check_clippy
        ;;
    test|tests)
        check_tests
        ;;
    docs|doc)
        check_docs
        ;;
    safety)
        check_safety
        ;;
    security)
        check_security
        ;;
    deps|dependencies)
        check_dependencies
        ;;
    wasm)
        check_wasm
        ;;
    bevy)
        check_bevy_free
        ;;
    coverage)
        check_coverage
        ;;
    all)
        run_all
        ;;
    *)
        echo "Usage: $0 [quick|fmt|clippy|test|docs|safety|security|deps|wasm|bevy|coverage|all]"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                    LOCAL CHECKS PASSED                        ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"

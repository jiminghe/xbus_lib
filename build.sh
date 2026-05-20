#!/usr/bin/env bash
# Build helper for the xbus_lib project.
#
# Usage:
#   ./build.sh            # build (default)
#   ./build.sh build      # build (incremental)
#   ./build.sh clean      # remove build dirs
#   ./build.sh rebuild    # clean, then build from scratch
#
# Environment:
#   DEBUG_HEX=1           # print raw TX/RX hex on stderr (defines DEBUG)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$ROOT/build"
TEST_BUILD_DIR="$ROOT/test/build"

cmake_flags=()
if [[ "${DEBUG_HEX:-0}" == "1" ]]; then
    cmake_flags+=("-DDEBUG_HEX=ON")
fi

do_clean() {
    rm -rf "$BUILD_DIR" "$TEST_BUILD_DIR"
    echo "Cleaned: $BUILD_DIR"
    echo "         $TEST_BUILD_DIR"
}

do_build() {
    cmake -S "$ROOT"      -B "$BUILD_DIR"      "${cmake_flags[@]}"
    cmake --build "$BUILD_DIR" -j

    cmake -S "$ROOT/test" -B "$TEST_BUILD_DIR" "${cmake_flags[@]}"
    cmake --build "$TEST_BUILD_DIR" -j
}

cmd="${1:-build}"
case "$cmd" in
    build)   do_build ;;
    clean)   do_clean ;;
    rebuild) do_clean; do_build ;;
    -h|--help|help)
        sed -n '2,9p' "$0"
        ;;
    *)
        echo "Unknown command: $cmd" >&2
        echo "Usage: $0 [build|clean|rebuild]" >&2
        exit 1
        ;;
esac

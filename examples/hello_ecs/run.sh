#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TARGET_NAME="$(basename "$SCRIPT_DIR")"

if [ ! -f "$SCRIPT_DIR/build/build.ninja" ]; then
    rm -rf "$SCRIPT_DIR/build"
    cmake -B "$SCRIPT_DIR/build" -G Ninja -S "$SCRIPT_DIR" \
        -DCMAKE_PREFIX_PATH="$REPO_ROOT/build" \
        -DLili2D_DIR="$REPO_ROOT/build"
fi
cmake --build "$SCRIPT_DIR/build"

cd "$SCRIPT_DIR/build" && ./"$TARGET_NAME" "$@"

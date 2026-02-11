#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

"$ROOT_DIR/scripts/bootstrap_deps.sh"

GENERATOR="${GENERATOR:-Visual Studio 17 2022}"
PLATFORM="${PLATFORM:-x64}"
BUILD_DIR="${BUILD_DIR:-build}"
CONFIG="${CONFIG:-Release}"

echo "[build] configuring with generator: $GENERATOR ($PLATFORM)"
cmake -S . -B "$BUILD_DIR" -G "$GENERATOR" -A "$PLATFORM"

echo "[build] building config: $CONFIG"
cmake --build "$BUILD_DIR" --config "$CONFIG"

PACKAGE_DIR="$BUILD_DIR/release-package"
rm -rf "$PACKAGE_DIR"
mkdir -p "$PACKAGE_DIR"

cp -r "$BUILD_DIR/artifacts/$CONFIG" "$PACKAGE_DIR/artifacts"
cp -r "$BUILD_DIR/driver_01spacecalibrator" "$PACKAGE_DIR/driver_01spacecalibrator"

ARCHIVE_PATH="$BUILD_DIR/ovrsc-release-$CONFIG.zip"
rm -f "$ARCHIVE_PATH"
cmake -E tar cf "$ARCHIVE_PATH" --format=zip -C "$PACKAGE_DIR" .

echo "[build] release archive created: $ARCHIVE_PATH"

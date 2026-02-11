#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[bootstrap] syncing submodule metadata..."
git submodule sync --recursive || true

echo "[bootstrap] fetching dependency submodules..."
git submodule update --init --recursive || true

ensure_repo() {
  local path="$1"
  local url="$2"
  if [[ ! -d "$path/.git" ]]; then
    if [[ -d "$path" ]]; then
      rm -rf "$path"
    fi
    echo "[bootstrap] cloning $path from $url"
    git clone --depth 1 "$url" "$path"
  fi
}

ensure_repo "lib/minhook" "https://github.com/bdunderscore/minhook.git"
ensure_repo "lib/glfw" "https://github.com/glfw/glfw.git"
ensure_repo "lib/imgui" "https://github.com/ocornut/imgui.git"
ensure_repo "lib/implot" "https://github.com/epezent/implot.git"
ensure_repo "lib/openvr" "https://github.com/ValveSoftware/openvr.git"

echo "[bootstrap] dependency bootstrap complete."

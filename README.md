# OpenVR Space Calibrator (OVRSC)

This repository contains a fork of OpenVR Space Calibrator with an overlay application and a SteamVR driver.

## Build status in this environment

This container does not include all third-party vendored dependencies required by this repo (`lib/minhook`, `lib/glfw`, `lib/imgui`, `lib/implot`), so a full binary build cannot be completed here until those directories are present.

## Expected local build flow (Windows)

1. Ensure required third-party libraries are present under `lib/`:
   - `minhook`
   - `glfw`
   - `imgui`
   - `implot`
2. Configure with CMake.
3. Build `Release`.
4. Package `build/artifacts/Release` and `build/driver_01spacecalibrator` for distribution.

## Notes

- Driver sources are in `src/driver`.
- Overlay sources are in `src/overlay`.
- Shared protocol types are in `src/common/Protocol.h`.

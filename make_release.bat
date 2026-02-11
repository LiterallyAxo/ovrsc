@echo off
setlocal enabledelayedexpansion

set ROOT_DIR=%~dp0
cd /d "%ROOT_DIR%"

echo [bootstrap] syncing submodule metadata...
git submodule sync --recursive

echo [bootstrap] fetching dependency submodules...
git submodule update --init --recursive

call :ensure_repo lib\minhook https://github.com/bdunderscore/minhook.git || exit /b 1
call :ensure_repo lib\glfw https://github.com/glfw/glfw.git || exit /b 1
call :ensure_repo lib\imgui https://github.com/ocornut/imgui.git || exit /b 1
call :ensure_repo lib\implot https://github.com/epezent/implot.git || exit /b 1
call :ensure_repo lib\openvr https://github.com/ValveSoftware/openvr.git || exit /b 1

echo [build] configuring Release (VS2022 x64)...
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 || exit /b 1

echo [build] building Release...
cmake --build build --config Release || exit /b 1

if exist build\release-package rmdir /s /q build\release-package
mkdir build\release-package || exit /b 1
xcopy /e /i /y build\artifacts\Release build\release-package\artifacts >nul || exit /b 1
xcopy /e /i /y build\driver_01spacecalibrator build\release-package\driver_01spacecalibrator >nul || exit /b 1

if exist build\ovrsc-release-Release.zip del /q build\ovrsc-release-Release.zip
cmake -E tar cf build\ovrsc-release-Release.zip --format=zip -C build\release-package . || exit /b 1

echo [done] build\ovrsc-release-Release.zip
endlocal
exit /b 0

:ensure_repo
set DEP_PATH=%~1
set DEP_URL=%~2
if not exist "%DEP_PATH%\.git" (
  if exist "%DEP_PATH%" rmdir /s /q "%DEP_PATH%"
  echo [bootstrap] cloning %DEP_PATH% from %DEP_URL%
  git clone --depth 1 "%DEP_URL%" "%DEP_PATH%" || exit /b 1
)
exit /b 0

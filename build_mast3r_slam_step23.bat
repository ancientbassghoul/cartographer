@echo off
REM ============================================================================
REM  Resume MASt3R-SLAM build: steps 2-3 (step 1 mast3r+curope already installed).
REM  - in3d installed with --no-deps to skip pyimgui (GUI-only; no Win/py3.11 wheel,
REM    and we use our own OpenCV/Rerun visualizer instead of mast3r_slam.visualization).
REM  - main package built --no-deps to KEEP numpy 2.4.4 (our VLM stack), then we add
REM    only the deps we actually need. Skips pyrealsense2 (RealSense input) and evo (eval).
REM  NO SILENT FALLBACKS: stops on first failing step.
REM ============================================================================
setlocal

call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" -vcvars_ver=14.36
if errorlevel 1 ( echo [FATAL] vcvars64 failed & exit /b 1 )

set "CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.1"
set "CUDA_HOME=%CUDA_PATH%"
set "PATH=%CUDA_PATH%\bin;%PATH%"
set "DISTUTILS_USE_SDK=1"
set "TORCH_CUDA_ARCH_LIST=8.6"
set "MAX_JOBS=8"

set "PY=D:\EXTEND\C2_SIM\XLAB\cartographer\venv\Scripts\python.exe"
cd /d "D:\EXTEND\C2_SIM\XLAB\cartographer\third_party\MASt3R-SLAM"

echo ============================== STEP 2: in3d (no GUI deps) ==============================
"%PY%" -m pip install --no-build-isolation --no-deps -e thirdparty/in3d
if errorlevel 1 ( echo [FATAL] step2 in3d failed & exit /b 1 )

echo ============================== STEP 3: backend CUDA compile (no deps) ==============================
"%PY%" -m pip install --no-build-isolation --no-deps -e .
if errorlevel 1 ( echo [FATAL] step3 backend compile failed & exit /b 1 )

echo ============================== VERIFY backend ==============================
"%PY%" -c "import torch, mast3r_slam_backends; print('mast3r_slam_backends OK | torch', torch.__version__, 'cuda', torch.cuda.is_available())"
if errorlevel 1 ( echo [FATAL] mast3r_slam_backends import failed & exit /b 1 )

echo ============================== STEP 4: runtime deps we need ==============================
"%PY%" -m pip install plyfile natsort
if errorlevel 1 ( echo [FATAL] step4 plyfile/natsort failed & exit /b 1 )
echo --- lietorch (CUDA compile from git) ---
"%PY%" -m pip install --no-build-isolation "lietorch @ git+https://github.com/princeton-vl/lietorch.git"
if errorlevel 1 ( echo [FATAL] step4 lietorch failed & exit /b 1 )

echo ============================== VERIFY numpy intact ==============================
"%PY%" -c "import numpy; print('numpy', numpy.__version__)"

echo ============================== BUILD OK ==============================
endlocal

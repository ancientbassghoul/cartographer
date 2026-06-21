@echo off
REM ============================================================================
REM  Build MASt3R-SLAM (windows branch) into the unified cartographer venv.
REM  Pins MSVC toolset 14.36 (= VS2022 17.6, MSVC 19.36) which nvcc 12.1 accepts;
REM  the newer 14.51 toolset would be rejected by CUDA 12.1.
REM  NO SILENT FALLBACKS: this script stops on the first failing step.
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

echo ============================== TOOLCHAIN ==============================
where cl
cl 2>&1 | findstr /C:"Version"
nvcc --version | findstr /C:"release"
"%PY%" -c "import torch; print('torch', torch.__version__, 'cuda', torch.version.cuda, torch.cuda.is_available())"

echo ============================== STEP 0: build tools ==============================
REM curope's setup.py imports torch at build time, so build isolation must be OFF
REM for every source build below (else pip's clean build-env has no torch).
"%PY%" -m pip install -U setuptools wheel
if errorlevel 1 ( echo [FATAL] step0 build tools failed & exit /b 1 )

echo ============================== STEP 1: mast3r ==============================
"%PY%" -m pip install --no-build-isolation -e thirdparty/mast3r
if errorlevel 1 ( echo [FATAL] step1 mast3r failed & exit /b 1 )

echo ============================== STEP 2: in3d ==============================
"%PY%" -m pip install --no-build-isolation -e thirdparty/in3d
if errorlevel 1 ( echo [FATAL] step2 in3d failed & exit /b 1 )

echo ============================== STEP 3: backends (CUDA compile) ==============================
"%PY%" -m pip install --no-build-isolation -e .
if errorlevel 1 ( echo [FATAL] step3 backend compile failed & exit /b 1 )

echo ============================== VERIFY torch intact ==============================
"%PY%" -c "import torch; print('torch', torch.__version__, 'cuda_ok', torch.cuda.is_available())"

echo ============================== BUILD OK ==============================
endlocal

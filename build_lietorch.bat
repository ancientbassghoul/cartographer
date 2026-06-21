@echo off
REM ============================================================================
REM  Clean-rebuild lietorch's CUDA extension into the unified cartographer venv.
REM  Same sanctioned toolchain as build_mast3r_slam*.bat (MSVC 14.36 + CUDA 12.1,
REM  TORCH_CUDA_ARCH_LIST=8.6). Force a fresh compile from the pinned commit to
REM  rule out a stale / ABI-mismatched build as the cause of the CUDA group-op
REM  access violation (Sim3.inv()/mul on cuda).
REM  The const->non-const kernel-pointer fix lives in third_party/lietorch already. If that dir is
REM  ever wiped, re-clone the pinned commit and re-apply lietorch_windows_const_fix.patch first:
REM    git clone --recursive https://github.com/princeton-vl/lietorch.git third_party/lietorch
REM    cd third_party/lietorch && git checkout e7df86554156b36846008d8ddbcc4d8521a16554
REM    git apply ..\..\lietorch_windows_const_fix.patch
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

echo ============================== TOOLCHAIN ==============================
where cl
nvcc --version | findstr /C:"release"
"%PY%" -c "import torch; print('torch', torch.__version__, 'cuda', torch.version.cuda, 'arch', torch.cuda.get_arch_list())"

echo ============================== REBUILD lietorch (from LOCAL clone, force, no cache) ==============================
REM Build from third_party/lietorch so we compile the (optionally patched) local source,
REM not a fresh git clone. eigen submodule is already checked out under that dir.
cd /d "D:\EXTEND\C2_SIM\XLAB\cartographer\third_party\lietorch"
"%PY%" -m pip install --no-build-isolation --no-deps --force-reinstall --no-cache-dir .
if errorlevel 1 ( echo [FATAL] lietorch rebuild failed & exit /b 1 )

echo ============================== VERIFY torch/numpy intact ==============================
"%PY%" -c "import torch, numpy; print('torch', torch.__version__, 'numpy', numpy.__version__, 'cuda_ok', torch.cuda.is_available())"

echo ============================== REBUILD OK ==============================
endlocal

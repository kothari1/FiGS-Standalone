#!/usr/bin/env bash
# Conda-first minimal installer that adheres to:
#  - Torch 2.1.2 with CUDA 11.8
#  - numpy == 1.26.4
#
# References:
#  - Nerfstudio installation recommends Torch 2.1.2 + CUDA 11.8.
#  - NumPy 1.26.4 release
#
set -eo pipefail

ENV_NAME="FiGS"
PYTHON_VERSION="3.10"
NUMPY_VERSION="1.26.4"
TORCH_VER="2.1.2"
TORCHVISION_VER="0.16.2"
TORCHAUDIO_VER="2.1.2"   # torchaudio pair; acceptable to install alongside
CUDA_MAJOR_MINOR="11.8"
TINY_CUDA_NN_REPO="git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch"

# Help
if [[ "${1-}" == "-h" ]] || [[ "${1-}" == "--help" ]]; then
  cat <<EOF
Usage: $0

This script creates a conda env named '${ENV_NAME}' with:
  - python ${PYTHON_VERSION}
  - numpy ${NUMPY_VERSION}
  - pytorch ${TORCH_VER} + CUDA ${CUDA_MAJOR_MINOR} (via conda channels)
  - tiny-cuda-nn (pip from NVlabs repo)
  - nerfstudio (pip)
EOF
  exit 0
fi

echo "=========================================="
echo "STEP 0: Install System Dependencies"
echo "=========================================="
echo ""

# Detect and install required system dependencies
if command -v apt-get &> /dev/null; then
    echo "=== Detected apt-based system (Debian/Ubuntu) ==="
    echo "=== Installing system build dependencies ==="
    
    # Install libcrypt/libxcrypt depending on Ubuntu version
    if apt-cache search libxcrypt-dev 2>/dev/null | grep -q libxcrypt-dev; then
        echo "Installing libxcrypt-dev (Ubuntu 22.04+)..."
        sudo apt-get update && sudo apt-get install -y libxcrypt-dev
    else
        echo "Installing libcrypt-dev (Ubuntu 24.04+)..."
        sudo apt-get update && sudo apt-get install -y libcrypt-dev
    fi
    
    # Install compiler toolchain and glibc headers
    # Note: We explicitly rely on system GCC to avoid sysroot conflicts with Conda GCC
    sudo apt-get install -y build-essential pkg-config gcc-11 g++-11 linux-libc-dev binutils
    
    echo "=== System dependencies installed successfully ==="
else
    echo "WARNING: apt-get not found. Skipping automatic system dependency installation."
    echo "Please manually install: libxcrypt-dev (or libcrypt-dev), build-essential, pkg-config, gcc-11, g++-11"
fi

echo ""
echo "=========================================="
echo "STEP 1: Nerfstudio Base Installation"
echo "=========================================="
echo ""

command -v conda >/dev/null 2>&1 || { echo "ERROR: conda not found in PATH. Install Miniconda/Anaconda first."; exit 1; }

# initialize conda for this shell (works for bash/zsh)
eval "$(conda shell.bash hook)"

echo "=== Creating conda env: ${ENV_NAME} (python ${PYTHON_VERSION}) ==="
# FIX: Removed gxx_linux-64 and gcc_linux-64 from here to prevent 'bits/timesize.h' error
conda create -n "${ENV_NAME}" -y python=="${PYTHON_VERSION}" numpy=="${NUMPY_VERSION}"

echo "=== Activating ${ENV_NAME} ==="
conda activate "${ENV_NAME}"

echo "=== STEP 1.5: Fix Build Tools (Symlinks) ==="
# FIX: Create symlinks to trick build systems into using system tools instead of missing conda tools
# This fixes the "x86_64-conda-linux-gnu-strip: not found" error during fpsample build
echo "Creating symlinks for build tools..."
ln -sf /usr/bin/strip "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-strip"
ln -sf /usr/bin/ar "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-ar"
ln -sf /usr/bin/ld "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-ld"
ln -sf /usr/bin/nm "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-nm"
ln -sf /usr/bin/ranlib "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-ranlib"

echo "=== Setting system compilers to avoid glibc header conflicts ==="
export CC="/usr/bin/gcc-11"
export CXX="/usr/bin/g++-11"
export CUDAHOSTCXX="/usr/bin/g++-11"
export CPATH="/usr/include:${CPATH}"

echo "=== Upgrading pip inside env ==="
python -m pip install --upgrade pip

conda env config vars set PYTHONNOUSERSITE=1
conda deactivate
conda activate "${ENV_NAME}"

echo "=== Installing PyTorch ${TORCH_VER} + CUDA ${CUDA_MAJOR_MINOR} ==="
pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118

echo "=== Creating constraints file for critical dependencies ==="
cat > /tmp/constraints.txt <<EOF
numpy==${NUMPY_VERSION}
torch==2.1.2+cu118
torchvision==0.16.2+cu118
EOF

echo "=== Installing CUDA Toolkit ==="
conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit -y

echo "=== Installing tiny-cuda-nn (torch bindings) ==="
# Set library paths for CUDA (for both runtime and linking)
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${CONDA_PREFIX}/lib/stubs:${LD_LIBRARY_PATH}"
export LIBRARY_PATH="${CONDA_PREFIX}/lib:${CONDA_PREFIX}/lib/stubs:${LIBRARY_PATH}"
export LDFLAGS="-L${CONDA_PREFIX}/lib/stubs -L${CONDA_PREFIX}/lib ${LDFLAGS}"

# FIX: Explicitly set CUDA_HOME to the conda environment
export CUDA_HOME="${CONDA_PREFIX}"

# Add NVCC flags for CUDA 11.8 compatibility
export NVCC_APPEND_FLAGS="-allow-unsupported-compiler"

# Detect GPU compute capability or use default
echo "Detecting GPU compute capability..."
COMPUTE_CAP=$(python -c "
import torch
try:
    if torch.cuda.is_available():
        compute_cap = torch.cuda.get_device_capability(0)
        arch = f'{compute_cap[0]}{compute_cap[1]}'
        print(arch)
    else:
        print('86')  # Default to Ada architecture (RTX 4090, etc)
except:
    print('86')  # Fallback to Ada architecture
" 2>/dev/null)

if [ -z "$COMPUTE_CAP" ]; then
  COMPUTE_CAP="86"
fi

echo "Using CUDA compute capability: $COMPUTE_CAP"
export TCNN_CUDA_ARCHITECTURES="$COMPUTE_CAP"

# Use local copy instead of cloning from GitHub
TCNN_LOCAL_PATH="/home/cyrus/workspaces/StanfordMSL/tiny-cuda-nn/bindings/torch"
if [ -d "$TCNN_LOCAL_PATH" ]; then
  echo "Using local tiny-cuda-nn at: $TCNN_LOCAL_PATH"
  python -m pip install ninja "$TCNN_LOCAL_PATH" --no-build-isolation
else
  echo "Local copy not found, installing from GitHub with custom compiler flags..."
  pip install ninja
  python -m pip install "${TINY_CUDA_NN_REPO}" --no-build-isolation
fi

echo "=== Installing COLMAP ==="
conda install -y -c conda-forge colmap

echo "=== Installing ffmpeg ==="
conda install -y -c conda-forge ffmpeg

if [ -d "./Hierarchical-Localization" ]; then
    echo "=== Installing Hierarchical-Localization ==="
    pip install --constraint /tmp/constraints.txt -e ./Hierarchical-Localization
fi

echo "=== Installing nerfstudio via pip ==="
# FIX: Ensure build isolation is disabled for complex builds if they fail
python -m pip install nerfstudio

# ns-install-cli

# Try to install CLI completions, but don't fail if pymeshlab/Qt issues occur
if ns-install-cli 2>&1 | head -20; then
  echo "CLI completions installed successfully"
else
  echo "WARNING: CLI completions failed, but nerfstudio is still functional"
  true  # Continue despite error
fi

echo
echo "=== Quick verification ==="
python - <<PY
import sys, importlib
import numpy as np
import torch
print("python:", sys.version.split()[0])
print("numpy:", np.__version__)
print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
try:
    print("cuda device count:", torch.cuda.device_count())
    print("cuda current device name:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "N/A")
except Exception as e:
    print("cuda info error:", e)
# sanity checks for versions (exit non-zero if mismatch)
if np.__version__ != "${NUMPY_VERSION}":
    print("ERROR: numpy version mismatch (expected ${NUMPY_VERSION}, got", np.__version__, ")")
    sys.exit(2)
if not torch.__version__.startswith("${TORCH_VER}"):
    print("WARNING: torch version does not start with ${TORCH_VER} -- installed:", torch.__version__)
PY

echo
echo "=== Nerfstudio base installation complete ==="

echo "=== Uninstalling JIT gsplat & reinstalling functioning version ==="
pip uninstall gsplat -y
pip install gsplat==1.4.0 --index-url https://docs.gsplat.studio/whl/pt21cu118

echo ""
echo "=========================================="
echo "Installing FiGS-specific dependencies"
echo "=========================================="
echo ""

echo "=== Installing misc dependencies ==="
# conda install -c conda-forge albumentations --freeze-installed
pip install albumentations --no-deps
conda install -y -c conda-forge qpsolvers
conda install -y -c conda-forge tabulate
conda install -y -c conda-forge cython 
pip install ipykernel --no-deps
pip install ipympl --no-deps
pip install rich imageio[ffmpeg]

# Install editable packages if they exist
if [ -d "../acados/interfaces/acados_template" ]; then
    echo "=== Installing acados_template ==="
    pip install -e ../acados/interfaces/acados_template
fi

echo "=== Installing FiGS (current package) ==="
pip install -e .

echo ""
echo "=========================================="
echo "Patching COLMAP parameter names"
echo "=========================================="
echo ""

# Fix deprecated SIFT parameter names in nerfstudio's colmap_utils.py
COLMAP_UTILS_PATH="${CONDA_PREFIX}/lib/python${PYTHON_VERSION}/site-packages/nerfstudio/process_data/colmap_utils.py"

if [ -f "$COLMAP_UTILS_PATH" ]; then
    echo "=== Patching COLMAP parameters in colmap_utils.py ==="
    # Replace SiftExtraction with FeatureExtraction
    sed -i 's/--SiftExtraction\.use_gpu/--FeatureExtraction.use_gpu/g' "$COLMAP_UTILS_PATH"
    # Replace SiftMatching with FeatureMatching
    sed -i 's/--SiftMatching\.use_gpu/--FeatureMatching.use_gpu/g' "$COLMAP_UTILS_PATH"
    echo "Successfully patched COLMAP parameters"
else
    echo "WARNING: Could not find colmap_utils.py at expected location: $COLMAP_UTILS_PATH"
fi
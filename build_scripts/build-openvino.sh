#!/usr/bin/env bash

# This script builds and installs OpenVINO 2022.3.1 on an ARM64 architecture.
# It performs the following steps:
# 1. Sets the DEBIAN_FRONTEND to noninteractive to avoid prompts during package installation.
# 2. Determines the directory of the script and sets it to the DIR variable.
# 3. Clones the OpenVINO repository from GitHub into the deps directory.
# 4. Updates and initializes the submodules within the OpenVINO repository.
# 5. Installs the build dependencies for OpenVINO.
# 6. Activates a Python virtual environment.
# 7. Installs the required Python packages for building OpenVINO.
# 8. Creates a build directory and navigates into it.
# 9. Configures the build with CMake, specifying various options and paths.
# 10. Compiles the OpenVINO source code using make with 4 parallel jobs.
# 11. Installs the compiled OpenVINO binaries.
# 12. Creates a tarball of the installed OpenVINO directory.
# 13. Creates symbolic links to the installed OpenVINO directory for easier access.
set -e

export DEBIAN_FRONTEND=noninteractive

export DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. >/dev/null 2>&1 && pwd)"

# Determine target architecture
TARGET_ARCH=$(dpkg --print-architecture)
if [ "$TARGET_ARCH" != "arm64" ] && [ "$TARGET_ARCH" != "amd64" ]; then
    echo "Unsupported architecture: $TARGET_ARCH. Exiting."
    exit 1
fi

if [ "$TARGET_ARCH" == "arm64" ]; then
    LIB_ARCH=aarch64-linux-gnu
else
    LIB_ARCH=x86_64-linux-gnu
fi

cd $DIR/deps/
git clone --depth 1 --branch 2022.3.1 https://github.com/openvinotoolkit/openvino.git
cd $DIR/deps/openvino
git submodule update --init --recursive
sudo ./install_build_dependencies.sh

python3 -m venv --prompt ov-build $DIR/deps/.venv
source $DIR/deps/.venv/bin/activate
pip3 install -U setuptools==65.6.1 wheel==0.38.1 "patchelf<=0.17.2.1; sys_platform == 'linux' and platform_machine == 'x86_64'" "Cython<3" pyyaml

mkdir -p build && cd build

NUM_CORES=$(($(lscpu | grep "^Core(s) per socket:" | awk '{print $4}') - 1))

cmake -DCMAKE_BUILD_TYPE=Release \
    -DOpenCV_DIR=/usr/lib/$LIB_ARCH/cmake/opencv4 \
    -DCMAKE_INSTALL_PREFIX=/opt/intel/openvino_2022.3.1 \
    -DENABLE_MKL_DNN=OFF \
    -DENABLE_CLDNN=OFF \
    -DENABLE_GNA=OFF \
    -DENABLE_SSE42=OFF \
    -DTHREADING=SEQ \
    -DENABLE_OPENCV=ON \
    -DNGRAPH_PYTHON_BUILD_ENABLE=OFF \
    -DNGRAPH_ONNX_IMPORT_ENABLE=ON \
    -DENABLE_PYTHON=ON \
    -DPYTHON_EXECUTABLE=$(which python3.10) \
    -DPYTHON_LIBRARY=/usr/lib/$LIB_ARCH/libpython3.10.so \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
    -DCMAKE_CXX_FLAGS=-latomic ..

export PYTHONWARNINGS=ignore:::setuptools.command.install
make -j$NUM_CORES
sudo make install

tar cvzf $DIR/dist/openvino_2022.3.1_${TARGET_ARCH}.tgz /opt/intel/openvino_2022.3.1 --owner=0 --group=0 --no-same-owner --no-same-permissions

sudo ln -sf /opt/intel/openvino_2022.3.1 /opt/intel/openvino_2022
sudo ln -sf /opt/intel/openvino_2022.3.1 /opt/intel/openvino

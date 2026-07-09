#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_file="$project_root/build_config.ini"
build_dir="$project_root/build"
vtk_source_dir="$project_root/3rd/VTK-9.6.1"
vtk_build_dir="$project_root/vtk_build"
vtk_install_dir="$vtk_build_dir/install"
vtk_config="$vtk_install_dir/lib/cmake/vtk-9.6/vtk-config.cmake"
generator="Visual Studio 16 2019"
platform="x64"
build_type="${1:-Release}"

get_ini_value() {
  local section="$1"
  local key="$2"

  awk -F= -v section="$section" -v key="$key" '
    $0 ~ "^[[:space:]]*\\[" section "\\][[:space:]]*$" { in_section=1; next }
    $0 ~ "^[[:space:]]*\\[" { in_section=0 }
    in_section {
      current_key=$1
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", current_key)
      if (current_key == key) {
        value=substr($0, index($0, "=") + 1)
        gsub(/^[[:space:]]+|[[:space:]\r]+$/, "", value)
        print value
        exit
      }
    }
  ' "$config_file"
}

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "Error: required file was not found: $1" >&2
    exit 1
  fi
}

require_directory() {
  if [[ ! -d "$1" ]]; then
    echo "Error: required directory was not found: $1" >&2
    exit 1
  fi
}

require_file "$config_file"

mv3drgbd_dev_root="$(get_ini_value MV3D dev_root)"
mv3drgbd_runtime_dir="$(get_ini_value MV3D rgbd_runtime_dir)"
mv3d_runtime_dir="$(get_ini_value MV3D common_runtime_dir)"

if [[ -z "$mv3drgbd_dev_root" || -z "$mv3drgbd_runtime_dir" || -z "$mv3d_runtime_dir" ]]; then
  echo "Error: build_config.ini must define all MV3D paths." >&2
  exit 1
fi

mv3d_required_files=(
  "$mv3drgbd_dev_root/Includes/Mv3dRgbdApi.h"
  "$mv3drgbd_dev_root/Includes/Mv3dRgbdDefine.h"
  "$mv3drgbd_dev_root/Includes/Mv3dRgbdImgProc.h"
  "$mv3drgbd_dev_root/Libraries/win64/Mv3dRgbd.lib"
  "$mv3drgbd_runtime_dir/Mv3dRgbd.dll"
  "$mv3drgbd_runtime_dir/MvUdp.dll"
  "$mv3drgbd_runtime_dir/Mv3dRgbdCfg.ini"
  "$mv3d_runtime_dir/pthreadVC2.dll"
)
for required_file in "${mv3d_required_files[@]}"; do
  require_file "$required_file"
done

require_directory "$vtk_source_dir"
require_file "$vtk_source_dir/CMakeLists.txt"

if [[ ! -f "$vtk_config" ]]; then
  echo "Configuring VTK..."
  cmake -S "$vtk_source_dir" -B "$vtk_build_dir" \
    -G "$generator" -A "$platform" \
    -DCMAKE_INSTALL_PREFIX="$vtk_install_dir" \
    -DBUILD_SHARED_LIBS=ON \
    -DVTK_BUILD_TESTING=OFF \
    -DVTK_BUILD_EXAMPLES=OFF \
    -DVTK_WRAP_PYTHON=OFF \
    -DVTK_GROUP_ENABLE_Qt=NO

  echo "Building and installing VTK..."
  cmake --build "$vtk_build_dir" --config Release --parallel
  cmake --build "$vtk_build_dir" --config Release --target INSTALL --parallel
else
  echo "Using installed VTK: $vtk_config"
fi

echo "Configuring test..."
cmake -S "$project_root" -B "$build_dir" \
  -G "$generator" -A "$platform" \
  -DVTK_DIR="$vtk_install_dir/lib/cmake/vtk-9.6" \
  -DMV3DRGBD_DEV_ROOT="$mv3drgbd_dev_root" \
  -DMV3DRGBD_RUNTIME_DIR="$mv3drgbd_runtime_dir" \
  -DMV3D_RUNTIME_DIR="$mv3d_runtime_dir" \
  -DBUILD_TESTING=ON

echo "Building test ($build_type)..."
cmake --build "$build_dir" --config "$build_type" --parallel

echo "Running tests ($build_type)..."
ctest --test-dir "$build_dir" -C "$build_type" --output-on-failure

echo "Build completed: $build_dir/bin/$build_type/test.exe"

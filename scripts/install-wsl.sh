#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=../versions.env
source "$root/versions.env"
deps=$(realpath -m -- "${1:-$root/.deps}")
if [ "${SKIP_APT:-0}" != 1 ]; then
  sudo apt-get update
  sudo apt-get install -y build-essential cmake ninja-build git pkg-config libeigen3-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav socat openssh-client python3 shellcheck clang-format
fi
mkdir -p "$deps"
source_dir="$deps/ProjectAirSim"
if [ ! -d "$source_dir/.git" ]; then
  git clone --filter=blob:none --no-checkout https://github.com/iamaisim/ProjectAirSim.git "$source_dir"
  git -C "$source_dir" sparse-checkout set --no-cone /client/cpp/ /core_sim/include/ \
    /core_sim/src/ /client/python/example_user_scripts/sim_config/ /docs/license.md \
    /LICENSE /CMakeLists.txt /client_version.txt
  git -C "$source_dir" checkout --detach "$PROJECTAIRSIM_COMMIT"
fi
if [ "$(git -C "$source_dir" rev-parse HEAD)" != "$PROJECTAIRSIM_COMMIT" ]; then
  echo 'ProjectAirSim differs from versions.env; use a fresh dependency directory.' >&2
  exit 1
fi
if [ -n "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]; then
  echo 'ProjectAirSim dependency contains local modifications.' >&2
  exit 1
fi
cmake -S "$source_dir/client/cpp" -B "$deps/cpp-build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$deps/cpp-sdk"
cmake --build "$deps/cpp-build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --build "$deps/cpp-build" --target ProjectAirSimCppClientMerged
cmake --install "$deps/cpp-build"
printf 'SDK ready: %s\n' "$deps/cpp-sdk"

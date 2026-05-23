#!/usr/bin/env bash
# Build vmx_lowlevel_driver on the VMX board (default: MinSizeRel / -Os).
#
# VMX has ~1 GB RAM, so we serialize (1 worker, sequential executor) and avoid the
# default colcon cache: stale -O0 / debug flags would otherwise sneak into the
# production binary. Run from the workspace root (the directory that contains src/).
#
# Usage:
#   bash src/vmx_lowlevel_driver/scripts/build_vmx_release.sh [Release|RelWithDebInfo|Debug|MinSizeRel]
#
# Any additional args are forwarded to colcon build.

set -euo pipefail

if [[ ! -d "${PWD}/src" ]]; then
  echo >&2 "ERROR: run from the workspace root containing src/ (e.g. /home/vmx/lowlevel_cpp_ws)."
  exit 1
fi
if [[ ! -d "${PWD}/src/vmx_lowlevel_driver" ]]; then
  echo >&2 "ERROR: src/vmx_lowlevel_driver not found in ${PWD}; deploy the package first."
  exit 1
fi

# Default build type: MinSizeRel (-Os). On the 1 GB VMX-Pi, full -O2 builds of
# this TU push cc1plus past 900 MB and the box thrashes its 1 GB swap for ~20
# minutes (sometimes OOM-killed). -Os keeps the toolchain under ~400 MB and the
# resulting binary is within a few % of -O2 for the small hot loops we have.
BUILD_TYPE="MinSizeRel"
if [[ $# -ge 1 && "${1:-}" =~ ^(Release|RelWithDebInfo|Debug|MinSizeRel)$ ]]; then
  BUILD_TYPE="$1"
  shift
fi

# Per-build-type CXX flag overrides. We always include
# `-fno-fat-lto-objects --param ggc-min-expand=10 --param ggc-min-heapsize=8192`
# to make gcc more aggressive about freeing template-instantiation garbage; this
# alone slashes peak cc1plus RSS by ~30% on Ubuntu-22.04/aarch64.
CXX_TUNING_FLAGS="--param ggc-min-expand=10 --param ggc-min-heapsize=8192"
case "${BUILD_TYPE}" in
  Release)        EXTRA_CXX="-O2 -DNDEBUG ${CXX_TUNING_FLAGS}" ;;
  RelWithDebInfo) EXTRA_CXX="-O2 -g -DNDEBUG ${CXX_TUNING_FLAGS}" ;;
  MinSizeRel)     EXTRA_CXX="-Os -DNDEBUG ${CXX_TUNING_FLAGS}" ;;
  Debug)          EXTRA_CXX="-O0 -g ${CXX_TUNING_FLAGS}" ;;
esac

# /opt/ros/*/setup.bash references unset internal vars that explode under
# `set -u`; relax just for the source line.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

exec colcon build \
  --parallel-workers 1 \
  --executor sequential \
  --packages-select vmx_lowlevel_driver \
  --cmake-clean-cache \
  --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" \
               "-DCMAKE_CXX_FLAGS_RELEASE=${EXTRA_CXX}" \
               "-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=${EXTRA_CXX}" \
               "-DCMAKE_CXX_FLAGS_MINSIZEREL=${EXTRA_CXX}" \
               "-DCMAKE_CXX_FLAGS_DEBUG=${EXTRA_CXX}" \
  --symlink-install \
  "$@"

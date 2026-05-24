#!/usr/bin/env bash
# Source this file to prepare a clean ROS 2 + Gazebo runtime shell.
#
# Usage:
#   source /home/ruben/halmstad_ws/scripts/setup_sim_env.sh [gpu|software|clean] [omnet]
#
# Modes:
#   gpu      - use WSLg / Mesa D3D12 acceleration, but do not force an adapter
#              unless LRS_SIM_GPU_ADAPTER is set.
#   software - force llvmpipe software OpenGL for Gazebo driver-timeout debugging.
#   clean    - only unset graphics overrides; do not select GPU or software mode.
#
# Add "omnet" as the second argument only for shells that need OMNeT++ commands.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "This script must be sourced, not executed."
    echo "Run: source /home/ruben/halmstad_ws/scripts/setup_sim_env.sh [gpu|software|clean] [omnet]"
    exit 2
fi

mode="${1:-gpu}"
with_omnet="${2:-${LRS_SIM_WITH_OMNET:-0}}"

export HALMSTAD_WS="${HALMSTAD_WS:-/home/ruben/halmstad_ws}"
export OMNET_WORKSPACE="${OMNET_WORKSPACE:-/home/ruben/omnet_workspace}"
export LRS_OMNET_PROJECT_ROOT="${LRS_OMNET_PROJECT_ROOT:-${OMNET_WORKSPACE}/UAV_UGV}"
export OMNETPP_SETENV="${OMNETPP_SETENV:-${OMNET_WORKSPACE}/omnetpp-6.2.0/setenv}"
export LRS_SIM_GPU_ADAPTER="${LRS_SIM_GPU_ADAPTER:-AMD Radeon RX 7600}"
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export EXP_BAGS="${EXP_BAGS:-${HALMSTAD_WS}/bags/experiments}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

_sim_path_remove_rocm() {
    local value="${1:-}"
    local part
    local out=()

    IFS=':' read -r -a _parts <<< "$value"
    for part in "${_parts[@]}"; do
        case "$part" in
            ""|/opt/rocm*|/usr/local/rocm*)
                ;;
            *)
                out+=("$part")
                ;;
        esac
    done

    local IFS=':'
    printf '%s' "${out[*]}"
}

# Remove ROCm/HIP variables that are useful for ML experiments but should not
# leak into Gazebo, ROS 2, or OMNeT++ runtime shells.
unset ROCM_HOME ROCM_PATH HIP_PATH HIP_PLATFORM HIP_VISIBLE_DEVICES
unset HSA_OVERRIDE_GFX_VERSION HSA_ENABLE_SDMA HSA_TOOLS_LIB
unset AMD_SERIALIZE_KERNEL PYTORCH_ROCM_ARCH HCC_AMDGPU_TARGET
unset MIOPEN_USER_DB_PATH MIOPEN_CUSTOM_CACHE_DIR GPU_MAX_HEAP_SIZE GPU_MAX_ALLOC_PERCENT

# VS Code / Python tooling may export this, which hides ~/.local packages from
# ROS Python nodes. The YOLO detector needs the user-site ultralytics install.
unset PYTHONNOUSERSITE

PATH="$(_sim_path_remove_rocm "${PATH:-}")"
LD_LIBRARY_PATH="$(_sim_path_remove_rocm "${LD_LIBRARY_PATH:-}")"
export PATH
if [[ -n "${LD_LIBRARY_PATH}" ]]; then
    export LD_LIBRARY_PATH
else
    unset LD_LIBRARY_PATH
fi

# Clear old GL/Mesa overrides before selecting the requested render mode.
unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset GALLIUM_DRIVER_LLVM
unset MESA_VK_WSI_PRESENT_MODE
unset vblank_mode
unset MESA_D3D12_DEFAULT_ADAPTER_NAME

case "$mode" in
    gpu)
        # Leave adapter selection to WSLg/Mesa unless the caller explicitly sets
        # LRS_SIM_GPU_ADAPTER, e.g. "AMD Radeon RX 7600".
        if [[ -n "${LRS_SIM_GPU_ADAPTER:-}" ]]; then
            export MESA_D3D12_DEFAULT_ADAPTER_NAME="$LRS_SIM_GPU_ADAPTER"
        fi
        export GALLIUM_DRIVER=d3d12
        ;;
    software|cpu|llvmpipe)
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
        ;;
    clean)
        ;;
    *)
        echo "Invalid sim env mode: $mode" >&2
        echo "Use: gpu, software, or clean" >&2
        return 2
        ;;
esac

if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "[simenv] Warning: conda env is active: ${CONDA_PREFIX}" >&2
    echo "[simenv] If ROS Python behaves oddly, run 'conda deactivate' and source simenv again." >&2
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
fi

if [[ -f "${HALMSTAD_WS}/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${HALMSTAD_WS}/install/setup.bash"
fi

if [[ -f "${HALMSTAD_WS}/src/lrs_halmstad/clearpath/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${HALMSTAD_WS}/src/lrs_halmstad/clearpath/setup.bash"
fi

case "$with_omnet" in
    1|true|yes|on|omnet)
        with_omnet=1
        ;;
    0|false|no|off|"")
        with_omnet=0
        ;;
    *)
        echo "Invalid OMNeT option: ${with_omnet}" >&2
        echo "Use second argument 'omnet' or set LRS_SIM_WITH_OMNET=1." >&2
        return 2
        ;;
esac

if [[ "$with_omnet" == "1" && -f "${OMNETPP_SETENV}" ]]; then
    _sim_had_nounset=0
    case "$-" in
        *u*) _sim_had_nounset=1 ;;
    esac
    set +u
    # shellcheck source=/dev/null
    source "${OMNETPP_SETENV}" >/dev/null
    if [[ "$_sim_had_nounset" == "1" ]]; then
        set -u
    else
        set +u
    fi
    unset _sim_had_nounset
elif [[ "$with_omnet" == "1" ]]; then
    echo "[simenv] Warning: OMNeT++ setenv not found: ${OMNETPP_SETENV}" >&2
fi

echo "[simenv] mode=${mode}"
echo "[simenv] HALMSTAD_WS=${HALMSTAD_WS}"
echo "[simenv] LRS_OMNET_PROJECT_ROOT=${LRS_OMNET_PROJECT_ROOT}"
echo "[simenv] OMNeT sourced=${with_omnet}"
echo "[simenv] GZ_VERSION=${GZ_VERSION}"
echo "[simenv] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[simenv] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
if [[ "${mode}" == "software" || "${mode}" == "cpu" || "${mode}" == "llvmpipe" ]]; then
    echo "[simenv] Rendering: software OpenGL via llvmpipe"
elif [[ "${mode}" == "clean" ]]; then
    echo "[simenv] Rendering: no explicit Mesa/OpenGL override"
elif [[ -n "${MESA_D3D12_DEFAULT_ADAPTER_NAME:-}" ]]; then
    echo "[simenv] Rendering: WSLg GPU, adapter=${MESA_D3D12_DEFAULT_ADAPTER_NAME}"
else
    echo "[simenv] Rendering: WSLg GPU, default adapter"
fi

# Usage: source scripts/env.sh
# Safe to source (does not exit your shell)

# Source ROS
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "ERROR: /opt/ros/jazzy/setup.bash not found"
  return 1
fi

# Workspace root (absolute)
export WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source overlay if built
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
else
  echo "WARN: $WS_ROOT/install/setup.bash not found. Run colcon build."
fi

# Keep DDS settings aligned with clearpath/robot.yaml defaults.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# WSL + FastDDS shared-memory transport often leaves stale lockfiles/ports and causes
# pervasive RTPS_TRANSPORT_SHM errors, delayed services, and inconsistent runtime
# behavior across repeated Gazebo/ROS relaunches. Disable SHM and use UDP transport
# by default for stable local testing (can be overridden by exporting 1 explicitly).
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

# Make Gazebo able to resolve local model:// URIs (e.g. matrice_100, zenmuse_z3)
# both from source and from the installed package tree.
_LRS_GZ_PATHS=(
  "$WS_ROOT/src/lrs_halmstad/models"
  "$WS_ROOT/install/lrs_halmstad/share"
)

for _p in "${_LRS_GZ_PATHS[@]}"; do
  if [ -d "$_p" ]; then
    case ":${GZ_SIM_RESOURCE_PATH:-}:" in
      *:"$_p":*) ;;
      *) export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$_p" ;;
    esac
    case ":${IGN_GAZEBO_RESOURCE_PATH:-}:" in
      *:"$_p":*) ;;
      *) export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH:+$IGN_GAZEBO_RESOURCE_PATH:}$_p" ;;
    esac
    case ":${GAZEBO_MODEL_PATH:-}:" in
      *:"$_p":*) ;;
      *) export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$_p" ;;
    esac
  fi
done

unset _LRS_GZ_PATHS
unset _p

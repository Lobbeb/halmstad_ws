echo "[cleanup] Killing follow_uav / leader_estimator / rosbag + clearing ROS graph..."

# 1) Kill by pattern (fast path)

pkill -9 -f "follow_uav" 2>/dev/null || true
pkill -9 -f "leader_estimator" 2>/dev/null || true
pkill -9 -f "ros2 bag record" 2>/dev/null || true
pkill -9 -f "install/lrs_halmstad/lib/lrs_halmstad/follow_uav" 2>/dev/null || true
pkill -9 -f "install/lrs_halmstad/lib/lrs_halmstad/leader_estimator" 2>/dev/null || true

sleep 1

# 2) Kill any remaining matching PIDs (guaranteed)

PIDS="$(ps aux | grep -E "follow_uav|leader_estimator|ros2 bag record" | grep -v grep | awk '{print $2}' | tr '\n' ' ')"
if [ -n "${PIDS// }" ]; then
echo "[cleanup] Still running PIDs: $PIDS -> killing -9"
kill -9 $PIDS 2>/dev/null || true
sleep 1
else
echo "[cleanup] OK: no matching processes ✅"
fi

# 3) Clear stale ROS graph (daemon reset)

echo "[cleanup] Resetting ros2 daemon (clears stale node listings)..."
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true
sleep 1

# Final verification

echo "[cleanup] Remaining matching processes:"
ps aux | grep -E "follow_uav|leader_estimator|ros2 bag record" | grep -v grep || echo "OK: none ✅"

echo "[cleanup] Remaining ROS nodes:"
ros2 node list | grep -E "^/follow_uav$|^/leader_estimator$" && echo "WARNING: still listed" || echo "OK: clean graph ✅"

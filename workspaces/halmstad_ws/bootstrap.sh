cat > bootstrap.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./bootstrap.sh
# Then:
#   source scripts/env.sh
#   colcon build --symlink-install

echo "[bootstrap] Installing Python deps (user site or venv)..."
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

# ROS2 deps: assumes ROS 2 is already installed on the machine.
# This installs missing apt deps for packages in src/.
if command -v rosdep >/dev/null 2>&1; then
  echo "[bootstrap] Running rosdep..."
  rosdep update || true
  rosdep install --from-paths src --ignore-src -r -y
else
  echo "[bootstrap] rosdep not found. Install it if you want automatic apt deps:"
  echo "  sudo apt update && sudo apt install -y python3-rosdep"
fi

echo "[bootstrap] Done."
EOF

chmod +x bootstrap.sh

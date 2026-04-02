#!/usr/bin/env bash
# debug_camera.sh — Diagnoses why camera_ros fails when launched from the dashboard.
# Run on the Pi: bash ~/makimate-2026/scripts/debug_camera.sh 2>&1 | tee /tmp/camera_debug.log

REPO="$HOME/makimate-2026"
SEP="━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

section() { echo; echo "$SEP"; echo "  $1"; echo "$SEP"; }

# ── 1. Device permissions ────────────────────────────────────────────────────
section "1. Camera devices"
ls -la /dev/video0 /dev/media0 2>/dev/null || echo "  (devices not found)"
echo "  Groups: $(id)"

# ── 2. Is camera_node already running? ──────────────────────────────────────
section "2. Existing camera processes"
ps aux | grep -E "camera_node|camera_ros" | grep -v grep || echo "  (none)"

# ── 3. Env inside plain bash -c ─────────────────────────────────────────────
section "3. LD_LIBRARY_PATH inside plain 'bash -c'"
bash -c 'echo "  LD_LIBRARY_PATH=$LD_LIBRARY_PATH"'

# ── 4. Env inside bash -lc (login shell — what the launch file uses) ─────────
section "4. LD_LIBRARY_PATH inside 'bash -lc' (login shell)"
bash -lc 'echo "  LD_LIBRARY_PATH=$LD_LIBRARY_PATH"'

# ── 5. Does maki_ws/setup.bash exist? ────────────────────────────────────────
section "5. maki_ws presence"
if [ -f "$HOME/maki_ws/install/setup.bash" ]; then
    echo "  FOUND: $HOME/maki_ws/install/setup.bash"
else
    echo "  MISSING: $HOME/maki_ws/install/setup.bash"
fi

# ── 6. LD_LIBRARY_PATH after sourcing exactly what the launch file does ──────
section "6. LD_LIBRARY_PATH after shell_prefix (what camera_ros sees)"
bash -lc "
  source /opt/ros/jazzy/setup.bash
  source $REPO/install/setup.bash
  { [ -f ~/maki_ws/install/setup.bash ] && source ~/maki_ws/install/setup.bash; true; }
  echo '  LD_LIBRARY_PATH='"\$LD_LIBRARY_PATH"
  echo '  AMENT_PREFIX_PATH='"\$AMENT_PREFIX_PATH" | tr ':' '\n' | head -10
"

# ── 7. Can ros2 find camera_ros package? ─────────────────────────────────────
section "7. ros2 pkg find camera_ros (via login shell + shell_prefix)"
bash -lc "
  source /opt/ros/jazzy/setup.bash
  source $REPO/install/setup.bash
  { [ -f ~/maki_ws/install/setup.bash ] && source ~/maki_ws/install/setup.bash; true; }
  ros2 pkg prefix camera_ros 2>&1 || echo '  FAILED — package not found'
"

# ── 8. Attempt camera_ros exactly as dashboard launches it (bash -lc + prefix)
section "8. Test: camera_ros via bash -lc with shell_prefix (5-second timeout)"
timeout 5 bash -lc "
  source /opt/ros/jazzy/setup.bash
  source $REPO/install/setup.bash
  { [ -f ~/maki_ws/install/setup.bash ] && source ~/maki_ws/install/setup.bash; true; }
  ros2 run camera_ros camera_node --ros-args \
    -p camera:=0 -p role:=video \
    -p sensor_mode:='640:480' \
    -p width:=640 -p height:=480 -p format:=BGR888
" 2>&1
echo "  exit code: $?"

# ── 9. Same test but without bash -lc (plain bash -c) ───────────────────────
section "9. Test: camera_ros via plain 'bash -c' (no login shell) (5s timeout)"
timeout 5 bash -c "
  source /opt/ros/jazzy/setup.bash
  source $REPO/install/setup.bash
  { [ -f ~/maki_ws/install/setup.bash ] && source ~/maki_ws/install/setup.bash; true; }
  ros2 run camera_ros camera_node --ros-args \
    -p camera:=0 -p role:=video \
    -p sensor_mode:='640:480' \
    -p width:=640 -p height:=480 -p format:=BGR888
" 2>&1
echo "  exit code: $?"

section "Done — share /tmp/camera_debug.log"

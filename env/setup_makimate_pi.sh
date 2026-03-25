#!/usr/bin/env bash
# =============================================================================
# MakiMate Pi5 Setup Script
# =============================================================================
# Target: Ubuntu 24.04 (noble) + ROS 2 Jazzy on Raspberry Pi 5
# Run this from the repo root:   bash env/setup_makimate_pi.sh
#
# What this does:
#   1. Installs system apt packages (env/apt_extra.txt)
#   2. Sets up Dynamixel + ReSpeaker udev rules
#   3. Installs ROS 2 dependencies via rosdep
#   4. Installs Python dependencies (system-wide, for ROS node compatibility)
#   5. Extracts the Piper TTS binary
#   6. Downloads Vosk ASR models
#   7. Builds the ROS workspace
# =============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"
echo "==> Working in: $REPO_DIR"

# ---- Check OS ---------------------------------------------------------------
if ! command -v lsb_release >/dev/null 2>&1; then
  echo "ERROR: lsb_release not found. Are you on Ubuntu?"
  exit 1
fi
CODENAME=$(lsb_release -sc)
if [[ "$CODENAME" != "noble" ]]; then
  echo "WARNING: designed for Ubuntu 24.04 (noble), got: $CODENAME"
fi

# ---- Helper -----------------------------------------------------------------
step() { echo; echo ">>> STEP $1/$TOTAL_STEPS: $2"; }
TOTAL_STEPS=7

# =============================================================================
# Step 1 — System apt packages
# =============================================================================
step 1 "Installing system apt packages"
sudo apt-get update -qq

APT_FILE="$REPO_DIR/env/apt_extra.txt"
if [[ ! -f "$APT_FILE" ]]; then
  echo "  WARNING: $APT_FILE not found — skipping apt extras."
else
  # Strip comments and blank lines
  mapfile -t APT_PKGS < <(grep -vE '^\s*#|^\s*$' "$APT_FILE")
  if [[ ${#APT_PKGS[@]} -gt 0 ]]; then
    sudo apt-get install -y "${APT_PKGS[@]}" || true
  fi
fi

# Additional packages needed by makimate (not always in apt_extra.txt)
sudo apt-get install -y \
  python3-pip \
  python3-venv \
  cmake \
  libdlib-dev \
  python3-opencv \
  || true

# =============================================================================
# Step 2 — udev rules (ReSpeaker + Dynamixel)
# =============================================================================
step 2 "Setting up udev rules"
bash "$REPO_DIR/scripts/install_respeaker_udev.sh"

# Dynamixel USB serial rule (ttyACM / ttyUSB access without sudo)
DXL_RULE="/etc/udev/rules.d/51-dynamixel.rules"
if [[ ! -f "$DXL_RULE" ]]; then
  echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE:="0666", GROUP="dialout"' \
    | sudo tee "$DXL_RULE" > /dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "  Dynamixel udev rule created."
fi

# Make sure user is in dialout group
if ! groups | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  echo "  Added $USER to dialout group (re-login to apply)."
fi

# =============================================================================
# Step 3 — ROS 2 dependencies via rosdep
# =============================================================================
step 3 "Installing ROS 2 package dependencies via rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init || true
fi
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

# =============================================================================
# Step 4 — Python dependencies (system-wide for ROS node compatibility)
# =============================================================================
step 4 "Installing Python dependencies"
# Use system Python so ROS nodes work without venv activation.
# --break-system-packages is required on Ubuntu 24.04.
sudo /usr/bin/python3 -m pip install \
  --break-system-packages \
  -r "$REPO_DIR/env/pi_requirements.txt"

# =============================================================================
# Step 5 — Piper TTS binary
# =============================================================================
step 5 "Extracting Piper TTS binary"
PIPER_DIR="$REPO_DIR/piper_bin"
PIPER_ARCHIVE="$PIPER_DIR/piper_linux_aarch64.tar.gz"
PIPER_BIN="$PIPER_DIR/piper/piper"

if [[ -f "$PIPER_BIN" ]]; then
  echo "  Piper already extracted at $PIPER_BIN"
elif [[ -f "$PIPER_ARCHIVE" ]]; then
  tar -xzf "$PIPER_ARCHIVE" -C "$PIPER_DIR"
  chmod +x "$PIPER_BIN" 2>/dev/null || true
  echo "  Piper extracted to $PIPER_DIR/piper/"
else
  echo "  WARNING: $PIPER_ARCHIVE not found."
  echo "  Download from: https://github.com/rhasspy/piper/releases"
  echo "  Place piper_linux_aarch64.tar.gz in $PIPER_DIR/"
fi

# =============================================================================
# Step 6 — Vosk ASR models
# =============================================================================
step 6 "Checking Vosk models"
VOSK_DIR="$HOME/vosk_models"
mkdir -p "$VOSK_DIR"

VOSK_ASR_MODEL="$VOSK_DIR/vosk-model-small-en-us-0.15"
VOSK_SPK_MODEL="$VOSK_DIR/vosk-model-spk-0.4"

if [[ ! -d "$VOSK_ASR_MODEL" ]]; then
  echo "  Downloading Vosk ASR model (small English, ~40 MB)..."
  wget -q --show-progress \
    "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip" \
    -O /tmp/vosk-asr.zip
  unzip -q /tmp/vosk-asr.zip -d "$VOSK_DIR"
  rm /tmp/vosk-asr.zip
  echo "  ASR model ready at $VOSK_ASR_MODEL"
else
  echo "  ASR model already present."
fi

if [[ ! -d "$VOSK_SPK_MODEL" ]]; then
  echo "  Downloading Vosk speaker model (~13 MB)..."
  wget -q --show-progress \
    "https://alphacephei.com/vosk/models/vosk-model-spk-0.4.zip" \
    -O /tmp/vosk-spk.zip
  unzip -q /tmp/vosk-spk.zip -d "$VOSK_DIR"
  rm /tmp/vosk-spk.zip
  echo "  Speaker model ready at $VOSK_SPK_MODEL"
else
  echo "  Speaker model already present."
fi

# =============================================================================
# Step 7 — Build ROS workspace
# =============================================================================
step 7 "Building ROS workspace"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install 2>&1 | tail -20

echo
echo "======================================================================"
echo " Setup complete!"
echo "======================================================================"
echo
echo " To start MakiMate, open a new terminal and run:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source $REPO_DIR/install/setup.bash"
echo "   ros2 launch makimate_bringup presentation_mode_launch_v2.py"
echo
echo " If this is a NEW robot, calibrate motors first:"
echo "   python3 $REPO_DIR/tools/calibrate_motors.py"
echo
echo " Dashboard (after launching MakiMate):"
echo "   http://$(hostname -I | awk '{print $1}'):8080"
echo
echo " NOTE: If you were added to the 'dialout' group, log out and back in"
echo "       before using the Dynamixel port."
echo "======================================================================"

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
TOTAL_STEPS=12

# =============================================================================
# Step 1 — ROS 2 Jazzy apt repository
# =============================================================================
step 1 "Adding ROS 2 Jazzy apt repository"
ROS_KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
ROS_LIST="/etc/apt/sources.list.d/ros2.list"
if [[ -f "$ROS_LIST" ]]; then
  echo "  ROS 2 apt repo already configured — skipping."
else
  sudo apt-get install -y curl
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o "$ROS_KEYRING"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=$ROS_KEYRING] \
http://packages.ros.org/ros2/ubuntu $CODENAME main" \
    | sudo tee "$ROS_LIST" > /dev/null
  echo "  ROS 2 repo added."
fi

# =============================================================================
# Step 2 — System apt packages
# =============================================================================
step 2 "Installing system apt packages"
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

# Core tools that must succeed — installed separately so a failure in
# apt_extra.txt (e.g. missing Docker repo) doesn't silently skip these.
sudo apt-get install -y \
  python3-pip \
  python3-venv \
  cmake \
  libdlib-dev \
  python3-opencv \
  python3-rosdep \
  python3-colcon-common-extensions \
  || true

# =============================================================================
# Step 3 — Hostname (maki.local mDNS)
# =============================================================================
step 3 "Setting hostname to 'maki' (accessible as maki.local)"
CURRENT_HOST=$(hostname)
if [[ "$CURRENT_HOST" != "maki" ]]; then
  sudo hostnamectl set-hostname maki
  # Update /etc/hosts so the new hostname resolves locally
  sudo sed -i "s/\b${CURRENT_HOST}\b/maki/g" /etc/hosts
  echo "  Hostname changed from '$CURRENT_HOST' to 'maki'."
  echo "  NOTE: Changes take effect after reboot."
else
  echo "  Hostname already 'maki' — skipping."
fi
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon

# =============================================================================
# Step 4 — udev rules (ReSpeaker + Dynamixel)
# =============================================================================
step 4 "Setting up udev rules"
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
# Step 5 — ROS 2 dependencies via rosdep
# =============================================================================
step 5 "Installing ROS 2 package dependencies via rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init || true
fi
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
  --skip-keys "makimate_interfaces"

# =============================================================================
# Step 6 — Python dependencies (system-wide for ROS node compatibility)
# =============================================================================
step 6 "Installing Python dependencies"
# Use system Python so ROS nodes work without venv activation.
# --break-system-packages is required on Ubuntu 24.04.
#
# Some packages (e.g. typing-extensions) are pre-installed by apt without a
# pip RECORD file, so pip can't uninstall them when upgrading.  Force-reinstall
# them via pip first so a proper RECORD file exists, then install everything else.
sudo /usr/bin/python3 -m pip install \
  --break-system-packages --ignore-installed \
  typing-extensions
sudo /usr/bin/python3 -m pip install \
  --break-system-packages \
  -r "$REPO_DIR/env/pi_requirements.txt"

# =============================================================================
# Step 7 — Piper TTS binary
# =============================================================================
step 7 "Extracting Piper TTS binary"
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
# Step 8 — Vosk ASR models
# =============================================================================
step 8 "Checking Vosk models"
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
# Step 9 — ALSA dsnoop config (shared mic access for Vosk + Whisper)
# =============================================================================
step 9 "Writing ~/.asoundrc (dsnoop for ReSpeaker shared capture)"
ASOUNDRC="$HOME/.asoundrc"
if [[ -f "$ASOUNDRC" ]] && grep -q "respeaker_shared" "$ASOUNDRC"; then
  echo "  ~/.asoundrc already contains respeaker_shared — skipping."
else
  # Append (or create) the dsnoop stanza.  Safe to append if file has other content.
  cat >> "$ASOUNDRC" << 'ASOUNDRC_EOF'

# MakiMate: shared ReSpeaker capture — lets Vosk and Whisper read simultaneously.
pcm.respeaker_shared {
    type dsnoop
    ipc_key 7890
    slave {
        pcm "hw:ArrayUAC10,0"
        channels 1
        rate 16000
        format S16_LE
        period_size 320
        buffer_size 6400
    }
}
ASOUNDRC_EOF
  echo "  Written respeaker_shared dsnoop to $ASOUNDRC"
fi

# =============================================================================
# Step 10 — Install systemd service (auto-start on boot)
# =============================================================================
step 10 "Installing makimate.service (auto-start on boot)"
chmod +x "$REPO_DIR/scripts/start_makimate.sh"
SERVICE_SRC="$REPO_DIR/env/makimate.service"
SERVICE_DST="/etc/systemd/system/makimate.service"
sed \
  -e "s|__USER__|$USER|g" \
  -e "s|__REPO_DIR__|$REPO_DIR|g" \
  "$SERVICE_SRC" | sudo tee "$SERVICE_DST" > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable makimate.service
echo "  makimate.service installed and enabled."
echo "  Start now:  sudo systemctl start makimate"
echo "  View logs:  journalctl -u makimate -f"

# =============================================================================
# Step 11 — sudoers: allow dashboard to restart makimate.service without password
# =============================================================================
step 11 "Configuring sudoers for makimate service restart"
SUDOERS_FILE="/etc/sudoers.d/makimate-restart"
echo "$USER ALL=(ALL) NOPASSWD: /bin/systemctl restart makimate.service" \
  | sudo tee "$SUDOERS_FILE" > /dev/null
sudo chmod 0440 "$SUDOERS_FILE"
echo "  Sudoers rule written to $SUDOERS_FILE"

# =============================================================================
# Step 12 — Build ROS workspace
# =============================================================================
step 12 "Building ROS workspace"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install 2>&1 | tail -20

echo
echo "======================================================================"
echo " Setup complete!"
echo "======================================================================"
echo
echo " MakiMate will start automatically on next boot (makimate.service)."
echo " To start it now without rebooting:"
echo "   sudo systemctl start makimate"
echo " View live logs:"
echo "   journalctl -u makimate -f"
echo " To start manually instead:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source $REPO_DIR/install/setup.bash"
echo "   ros2 launch maki_operational_nodes presentation_mode_v3.launch.py"
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

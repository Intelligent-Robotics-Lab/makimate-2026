#!/bin/bash
# MakiMate demo setup script.
# Opens all required SSH sessions and tunnels in tmux windows.
# Usage: ./scripts/setup_demo.sh
#
# Dependencies: sshpass, tmux
#   sudo apt install sshpass tmux

set -e

# ── Configuration ────────────────────────────────────────────────────────────
PI_HOST="141.210.88.112"
PI_USER="makimate"
PI_PASS="123"

SERVER_HOST="141.210.154.165"
SERVER_USER="maki"
SERVER_PASS="m@ki"   # ← set this before running

LLM_PORT=8004
ASR_PORT=8005
# ─────────────────────────────────────────────────────────────────────────────

# Auto-detect this machine's IP
LAPTOP_IP=$(hostname -I | awk '{print $1}')

# Check dependencies
for dep in sshpass tmux; do
    if ! command -v "$dep" &>/dev/null; then
        echo "ERROR: '$dep' not found. Install with: sudo apt install $dep"
        exit 1
    fi
done

if [ "$SERVER_PASS" = "FILL_IN_SERVER_PASSWORD" ]; then
    echo "ERROR: Set SERVER_PASS in this script before running."
    exit 1
fi

SSH_PI="sshpass -p $PI_PASS ssh -o StrictHostKeyChecking=no"
SSH_SRV="sshpass -p $SERVER_PASS ssh -o StrictHostKeyChecking=no"

SESSION="makimate-demo"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null || true

echo "Starting MakiMate demo setup..."
echo "Detected laptop IP: $LAPTOP_IP"
echo ""

# ── Window: robot ─────────────────────────────────────────────────────────────
# SSH into Pi and launch the ROS stack
tmux new-session -d -s $SESSION -n "robot"
tmux send-keys -t $SESSION:robot \
    "$SSH_PI $PI_USER@$PI_HOST 'source ~/makimate-2026/install/setup.bash && ros2 launch maki_operational_nodes presentation_mode_v3.launch.py'" \
    Enter

# ── Window: ollama ────────────────────────────────────────────────────────────
tmux new-window -t $SESSION -n "ollama"
tmux send-keys -t $SESSION:ollama \
    "$SSH_SRV $SERVER_USER@$SERVER_HOST '~/bin/ollama serve'" \
    Enter

# ── Window: llm-server ───────────────────────────────────────────────────────
tmux new-window -t $SESSION -n "llm-server"
tmux send-keys -t $SESSION:llm-server \
    "$SSH_SRV $SERVER_USER@$SERVER_HOST 'cd asr_llm_tts && source venv/bin/activate && python -m uvicorn server_llm:app --app-dir src --host 0.0.0.0 --port $LLM_PORT'" \
    Enter

# ── Window: asr-server ───────────────────────────────────────────────────────
tmux new-window -t $SESSION -n "asr-server"
tmux send-keys -t $SESSION:asr-server \
    "$SSH_SRV $SERVER_USER@$SERVER_HOST 'cd asr_llm_tts && source venv/bin/activate && python -m uvicorn server_asr:app --app-dir src --host 0.0.0.0 --port $ASR_PORT'" \
    Enter

# ── Window: tunnels ──────────────────────────────────────────────────────────
# Forward server ports to this machine so the Pi can reach them via laptop IP
tmux new-window -t $SESSION -n "tunnels"
tmux send-keys -t $SESSION:tunnels \
    "$SSH_SRV -N -L 0.0.0.0:$LLM_PORT:localhost:$LLM_PORT $SERVER_USER@$SERVER_HOST & $SSH_SRV -N -L 0.0.0.0:$ASR_PORT:localhost:$ASR_PORT $SERVER_USER@$SERVER_HOST & wait" \
    Enter

# ── Summary ───────────────────────────────────────────────────────────────────
echo "Done. Attach to tmux with: tmux attach -t $SESSION"
echo ""
echo "Dashboard : http://maki.local:8080/"
echo "LLM URL   : http://$LAPTOP_IP:$LLM_PORT"
echo "ASR URL   : http://$LAPTOP_IP:$ASR_PORT"
echo ""
echo "Recommended dashboard settings:"
echo "  Smoothing Alpha   = 0.08"
echo "  Noise Suppression = On"
echo "  Camera Offset L/R = 0.5"
echo "  Camera Offset U/D = 0.4"
echo "  Track Speed L/R   = 1.2"
echo "  ASR model         = base"

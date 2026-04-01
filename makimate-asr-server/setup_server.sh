#!/usr/bin/env bash
# =============================================================================
# MakiMate ASR Server — setup script
# =============================================================================
# Runs on a Mac or Linux machine with a capable CPU/GPU.
# Sets up a Python venv and installs faster-whisper + FastAPI.
#
# Usage:
#   cd makimate-asr-server
#   bash setup_server.sh
#
# Then start the server:
#   source venv/bin/activate
#   python server.py --host 0.0.0.0 --port 8002
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

VENV="$SCRIPT_DIR/venv"

echo "==> Setting up ASR server in: $SCRIPT_DIR"

# ---- Create venv if needed --------------------------------------------------
if [[ ! -d "$VENV" ]]; then
  echo ">>> Creating Python venv..."
  python3 -m venv "$VENV"
fi

source "$VENV/bin/activate"

echo ">>> Installing dependencies..."
pip install --upgrade pip -q
pip install -r requirements.txt

echo
echo "======================================================================"
echo " ASR server setup complete!"
echo "======================================================================"
echo
echo " Start the server:"
echo "   source $VENV/bin/activate"
echo "   python server.py --host 0.0.0.0 --port 8002"
echo
echo " The server will download the Whisper model on first transcription."
echo " Available models: tiny, base, small, medium, large-v3"
echo " Default is 'base'. Override with:  python server.py --model small"
echo
echo " Test it:"
echo "   curl http://localhost:8002/health"
echo
echo " Point the Pi dashboard at:  http://<your-ip>:8002"
echo "======================================================================"

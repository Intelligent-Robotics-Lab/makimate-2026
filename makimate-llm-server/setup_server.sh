#!/usr/bin/env bash
# =============================================================================
# MakiMate LLM Server — Linux/Mac setup script
# =============================================================================
# Usage:
#   git clone https://github.com/Intelligent-Robotics-Lab/makimate-2026.git
#   cd makimate-2026/makimate-llm-server
#   bash setup_server.sh
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

VENV="$SCRIPT_DIR/venv"
MODEL="qwen2.5:7b-instruct"

echo "==> Setting up LLM server in: $SCRIPT_DIR"

# ---- Install Ollama if missing -----------------------------------------------
if ! command -v ollama >/dev/null 2>&1; then
  echo ">>> Ollama not found — installing..."
  curl -fsSL https://ollama.com/install.sh | sh
else
  echo ">>> Ollama already installed: $(ollama --version)"
fi

# ---- Pull model --------------------------------------------------------------
echo ">>> Pulling model: $MODEL (this may take a while on first run)..."
OLLAMA_HOST=0.0.0.0 ollama pull "$MODEL"

# ---- Python venv + deps ------------------------------------------------------
if [[ ! -d "$VENV" ]]; then
  echo ">>> Creating Python venv..."
  python3 -m venv "$VENV"
fi

source "$VENV/bin/activate"
pip install --upgrade pip -q
pip install -r requirements_minimal.txt -q

echo
echo "======================================================================"
echo " Setup complete!"
echo "======================================================================"
echo
echo " Start the server:"
echo
echo "   Terminal 1 — start Ollama:"
echo "     OLLAMA_HOST=0.0.0.0 ollama serve"
echo
echo "   Terminal 2 — start the FastAPI server:"
echo "     source $VENV/bin/activate"
echo "     cd $SCRIPT_DIR/src"
echo "     uvicorn server_llm:app --host 0.0.0.0 --port 8000"
echo
echo " Find your IP:  hostname -I"
echo " Enter in the Pi dashboard under LLM URL: http://<your-ip>:8000"
echo
echo " Test it:"
echo "   curl -X POST http://localhost:8000/chat/stream \\"
echo "        -H 'Content-Type: application/json' \\"
echo "        -d '{\"message\": \"hello\"}'"
echo "======================================================================"

# =============================================================================
# MakiMate LLM Server — Windows setup script (PowerShell)
# =============================================================================
# Prerequisites (install these first if you don't have them):
#   - Git:    https://git-scm.com/download/win
#   - Python: https://www.python.org/downloads  (tick "Add Python to PATH")
#   - Ollama: https://ollama.com
#
# Usage — open PowerShell and run:
#   git clone https://github.com/Intelligent-Robotics-Lab/makimate-2026.git
#   cd makimate-2026\makimate-llm-server
#   .\setup_server.ps1
# =============================================================================

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Model = "qwen2.5:7b-instruct"

Set-Location $ScriptDir
Write-Host "==> Setting up LLM server in: $ScriptDir"

# ---- Check Ollama ------------------------------------------------------------
if (-not (Get-Command ollama -ErrorAction SilentlyContinue)) {
    Write-Host ""
    Write-Host "ERROR: Ollama is not installed or not on PATH." -ForegroundColor Red
    Write-Host "Download and install it from: https://ollama.com" -ForegroundColor Yellow
    Write-Host "Then re-run this script."
    exit 1
}
Write-Host ">>> Ollama found."

# ---- Set OLLAMA_HOST so the Pi can reach it ----------------------------------
$existing = [System.Environment]::GetEnvironmentVariable("OLLAMA_HOST", "Machine")
if ($existing -ne "0.0.0.0") {
    Write-Host ">>> Setting OLLAMA_HOST=0.0.0.0 (system-wide so Pi can connect)..."
    [System.Environment]::SetEnvironmentVariable("OLLAMA_HOST", "0.0.0.0", "Machine")
    $env:OLLAMA_HOST = "0.0.0.0"
    Write-Host "    Done. Ollama must be restarted for this to take effect." -ForegroundColor Yellow
} else {
    Write-Host ">>> OLLAMA_HOST already set to 0.0.0.0."
}

# ---- Pull model --------------------------------------------------------------
Write-Host ">>> Pulling model: $Model (this may take a while on first run)..."
ollama pull $Model

# ---- Python venv + deps ------------------------------------------------------
if (-not (Test-Path "$ScriptDir\venv")) {
    Write-Host ">>> Creating Python venv..."
    python -m venv venv
}

Write-Host ">>> Installing Python dependencies..."
& "$ScriptDir\venv\Scripts\pip.exe" install --upgrade pip -q
& "$ScriptDir\venv\Scripts\pip.exe" install -r requirements_minimal.txt -q

Write-Host ""
Write-Host "======================================================================" -ForegroundColor Green
Write-Host " Setup complete!" -ForegroundColor Green
Write-Host "======================================================================"
Write-Host ""
Write-Host " Start the server:"
Write-Host ""
Write-Host "   Terminal 1 — start Ollama (restart it if you just set OLLAMA_HOST):"
Write-Host "     ollama serve"
Write-Host ""
Write-Host "   Terminal 2 — start the FastAPI server:"
Write-Host "     cd $ScriptDir\src"
Write-Host "     ..\venv\Scripts\activate"
Write-Host "     uvicorn server_llm:app --host 0.0.0.0 --port 8000"
Write-Host "     (Allow firewall access when Windows prompts)"
Write-Host ""
Write-Host " Find your IP:  ipconfig"
Write-Host "   Look for 'IPv4 Address' under your active network adapter."
Write-Host " Enter in the Pi dashboard under LLM URL: http://<your-ip>:8000"
Write-Host "======================================================================"

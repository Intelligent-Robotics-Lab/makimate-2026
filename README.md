# MakiMate Winter 2026


# Table of Contents
[Setup](https://github.com/Intelligent-Robotics-Lab/makimate-2026/tree/main?tab=readme-ov-file#setup)

[Git Commands](https://github.com/Intelligent-Robotics-Lab/makimate-2026/edit/main/README.md#git-commands)

[Webpage](https://github.com/Intelligent-Robotics-Lab/makimate-2026/edit/main/README.md#webpage-dashboard)

[Server Setup](https://github.com/Intelligent-Robotics-Lab/makimate-2026/edit/main/README.md#server-setup)

---

# Setup
## **Raspberry Pi**
1) Flash Ubuntu 24.04 to the Pi 
2) `git clone https://github.com/Intelligent-Robotics-Lab/makimate-2026.git`
3) `cd ~/makimate-2026`
4) `bash env/setup_makimate_pi.sh`
5) `python3 tools/calibrate_motors.py`
6) Then, go to config/motor_limits.yaml and update limits if needed.
7) `sudo systemctl start makimate`

---

# Webpage Dashboard
The dashboard is used for real-time logging and modification of Makimate's ROS parameters. Additionally, you can change the LLM model and server URL to be used, along with changing what ASR model is used.
### Opening the Webpage
1) On your browser, type in `http://maki.local:8080`
2) In the case that your network blocks mDNS, type in `https://<pi-ip>:8080`
5) The <pi-ip> is obtained by running `hostname -I` in the Pi's terminal.

---

# Server Setup
Servers are required for LLM usage and can optionally be used to run larger ASR models as well. 

## Linux ASR Server Setup
1) Clone the repo: git clone `https://github.com/Intelligent-Robotics-Lab/makimate-2026.git`
3) Run `cd ~/makimate-2026/makimate-asr-server && bash setup_server.sh`
5) Run the server (base model): `python server.py`; note that if a different port is needed, use: `python server.py --port 8002`. To select a model, use `python server.py --model medium`.
6) Possible models to use: `tiny`, `base`, `small`, `medium`, `large-v3`, `distil-large-v3`
7) Port is 8001 (or whatever you choose). Find server ip: `hostname-I`
8) Test: `curl http://localhost:8001/health`
9) Open the Makimate dashboard and enter the Whisper URL: `http://<server-ip>:8001`
10) Pick the Whisper model the server is running (or a new one to swap).
11) Click "Apply Whisper".
12) Click "Check Server" to ensure that the server is operational and the correct ASR model is loaded.
13) To use the local Pi setup, clear the Whisper URL field and click "Apply Whisper".

## Linux LLM Server Setup
1) Clone the repo: `git clone https://github.com/Intelligent-Robotics-Lab/makimate-2026.git`
2) Run setup: `cd ~/makimate-2026/makimate-llm-server && bash setup_server.sh`
3) Start Ollama in one terminal: `OLLAMA_HOST=0.0.0.0 ollama serve`
4) Start the server in a second terminal:
      `cd ~/makimate-2026/makimate-llm-server/src`
      `source ../venv/bin/activate`
      `uvicorn server_llm:app --host 0.0.0.0 --port 8000`
5) Find your server IP: `hostname -I`
6) Test: `curl -X POST http://localhost:8000/chat/stream -H "Content-Type: application/json" -d '{"message": "hello"}'`
7) Enter the LLM URL in the Pi dashboard: `http://<server-ip>:8000`
8) Click "Apply LLM"

## Windows LLM Server Setup
1) Install pre-requisites if not already installed:
    - Git: https://git-scm.com/download/win
    - Python 3.10+: https://www.python.org/downloads — select "Add Python to PATH"
    - Ollama: https://ollama.com
2) Clone the repo. Open Powershell:
    `git clone https://github.com/Intelligent-Robotics-Lab/makimate-2026.git`
    `cd makimate-2026\makimate-llm-server`
3) Run setup: `.\setup_server.ps1`. If you get an execution policy error, first run: `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`
4) Restart Ollama after setup. In Powershell: `ollama serve`
5) Start the server in a second Powershell window:
    `cd makimate-2026\makimate-llm-server\src`
    `..\venv\Scripts\activate`
    `uvicorn server_llm:app --host 0.0.0.0 --port 8000`
6) Allow firewall access when prompted
7) Find your ip: `ipconfig`; look for IPv4 address.
8) Test: open a browser and go to http://localhost:8000/docs
9) Enter the LLM URL in the Pi dashboard: http://<your-ip>:8000
10) Click "Apply LLM"


---
# Project Structure
```
MakiMate
|-src
├── camera_ros
│   ├── launch
│   │   └── camera.launch.py
├── maki_operational_nodes
│   ├── launch
│   │   ├── demo_mode.launch.py
│   │   ├── docker_presentation_mode.launch.py
│   │   ├── full_feature_mode.launch.py
│   │   └── presentation_mode.launch.py
│   ├── maki_operational_nodes
│   │   ├── maki_awake_behavior.py
│   │   ├── maki_launch_manager.py
│   │   └── maki_operational_modes.py
├── makimate_asr
│   ├── makimate_asr
│   │   ├── ai_command_router.py
│   │   ├── asr_led_node.py
│   │   ├── natural_tts_node.py
│   │   ├── respeaker_vosk_asr.py
├── makimate_dxl
│   ├── makimate_dxl
│   │   ├── clear_hw_error.py
│   │   ├── dxl_dump_limits.py
│   │   ├── dxl_voltage_debug.py
│   │   ├── expressions.yaml
│   │   ├── maki_behavior.py
│   │   ├── maki_dxl_6.py
│   │   └── maki_expressions.py
├── makimate_vision
│   ├── makimate_vision
│   │   ├── face_to_maki.py
│   │   └── face_tracker_node.py
└── server_llm
    ├── launch
    │   └── llm_bridge.launch.py
    ├── server_llm
    │   ├── llm_bridge_node.py
    │   ├── say.py
    │   └── tty.py
```

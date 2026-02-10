# makimate-2026

## Setup
**Development Environment**
1) Install VScode: https://code.visualstudio.com/
2) Install Extension "Remote - SSH" by Microsoft: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh
   
**Raspberry Pi**
1) Install (if not already) and enable SSH: `sudo systemctl enable ssh`, `sudo systemctl start ssh`
2) Check status with `systemctl status ssh`
3) Type `hostname -I`

Now, in VScode, press `Ctrl+Shift+P` and search for `Remote-SSH: Connect to Host`. Enter `<name>@<pi-ip-address>` and enter the password (Pi authentication password).

---

## Useful Commands
- `git pull origin master`
- `git commit -m 'Some message about the change'`
- `git push origin master`
### Only commit changes in one folder: 
```
git add src/makimate_asr/
git commit -m "Updated ASR"
git push
```

---
## Project Structure
NOTE: Vosk models are local on the Pi, NOT in the repo. Instructions to download are beneath project structure.
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
---

## Vosk Model Installation (on the Pi)
1) In home/ folder, create a folder called `vosk_models`
2) In terminal, run:
```
cd ~/vosk_models
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
wget https://alphacephei.com/vosk/models/vosk-model-spk-0.4.zip
unzip vosk-model-spk-0.4.zip
unzip vosk-model-small-en-us-0.15.zip
```

## Package Installation (on the Pi)
# NOTE: This is a temporary solution. These should all be added as requirements in clean_requirements.txt st. you can install them in one command.
1) Pixelring: `env/asr_requirements.txt` --> https://pypi.org/project/pixel-ring/ download, add `pixel-ring-0.1.0 folder` to `makimate_asr`. Now in asr_requirements.txt, change to: `pixel-ring @ file:///home/emanuel/MakiMate2026/MakiMate/src/makimate_asr/makimate_asr/pixel-ring-0.1.0`
2) `source ~/asr_venv/bin/activate` then `pip install -r ~/asr_venv/clean_requirements.txt`
3) `~/asr_venv/bin/pip install vosk sounddevice numpy piper-tts onnxruntime requests`
4) `pip install jinja2 setuptools typeguard`
5) `pip install -r ~/asr_venv/clean_requirements.txt`
6) `pip install opencv-python-headless`
---

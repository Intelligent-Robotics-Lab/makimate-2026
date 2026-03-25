# makimate-2026


# Table of Contents
[Setup](https://github.com/Intelligent-Robotics-Lab/makimate-2026/tree/main?tab=readme-ov-file#setup)
[Git Commands](https://github.com/Intelligent-Robotics-Lab/makimate-2026/edit/main/README.md#git-commands)
[Webpage](https://github.com/Intelligent-Robotics-Lab/makimate-2026/edit/main/README.md#webpage-dashboard)

## Setup
**Raspberry Pi**
1) Open terminal on the Raspberry Pi
2) `git clone <repo> ~/makimate-2026`
3) `cd ~/makimate-2026`
4) `bash env/setup_makimate_pi.sh`
5) `python3 tools/calibrate_motors.py`
6) Then, go to config/motor_limits.yaml and update limits if needed.

**Development Environment**
1) Install VScode: https://code.visualstudio.com/
2) Install Extension "Remote - SSH" by Microsoft: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh
3) On the Raspberry Pi, install and enable SSH: `sudo systemctl enable ssh`, `sudo systemctl start ssh`
4) Check status with `systemctl status ssh`
5) Type `hostname -I`
6) In VScode, press `Ctrl+Shift+P` and search for `Remote-SSH: Connect to Host`. Enter `<name>@<pi-ip-address>` and enter the password (Pi authentication password).

---

## Git Commands
- `git pull origin master`
- `git commit -m 'Some message about the change'`
- `git push origin master`
### To commit changes in one folder: 
```
git add src/makimate_asr/
git commit -m "Updated ASR (example)"
git push
```

## Webpage Dashboard
The dashboard is used for real-time logging and modification of Makimate's ROS parameters. Additionally, you can change the LLM model and server URL to be used, along with changing what ASR model is used.
### Opening the Webpage
1) In the Pi's terminal, run the following two commands:
2) `source ~/makimate-2026/install/setup.bash`
3) `ros2 run makimate_dashboard dashboard`
4) On your browser, type in `http://<pi-ip>:8080`


---
## Project Structure
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

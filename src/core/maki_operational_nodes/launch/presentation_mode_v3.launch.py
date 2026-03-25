from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    shell_prefix = (
        "source /opt/ros/jazzy/setup.bash && "
        "source ~/makimate-2026/install/setup.bash && "
    )

    # 1a) ASR text: Whisper with VAD-based utterance segmentation.
    #     model_size options: tiny (~0.3s latency), base (~0.5s), small (~1s)
    #     For server offload: set server_url:='http://<mac-ip>:8001'
    #     and run makimate-asr-server/server.py on the Mac/school machine.
    whisper_asr_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr respeaker_whisper_asr "
              "--ros-args "
              "-p device:=0 "
              "-p model_size:=base "
              "-p asr_topic:=/asr/text "
              "-p enable_topic:=/asr/enable "
              "-p server_url:='' "
              "-p no_speech_threshold:=0.6 "
              "-p vad_aggressiveness:=2 "
              "-p silence_ms:=400"
        ],
        output="screen",
    )

    # 1b) Speaker embeddings: Vosk with text publishing OFF.
    #     Provides /voice/speaker_embedding → speaker_recognition_node.
    vosk_embeddings_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr respeaker_vosk_asr "
              "--ros-args "
              "-p sample_rate:=16000.0 "
              "-p device:=0 "
              "-p model_path:=$HOME/vosk_models/vosk-model-small-en-us-0.15 "
              "-p spk_model_path:=$HOME/vosk_models/vosk-model-spk-0.4 "
              "-p publish_text:=false "
              "-p publish_llm:=false "
              "-p asr_topic:=/asr/text "
              "-p enable_topic:=/asr/enable"
        ],
        output="screen",
    )

    # 2) Voice Calibration Node
    voice_calibration_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr voice_calibration "
              "--ros-args "
              "-p vosk_model_path:=$HOME/vosk_models/vosk-model-small-en-us-0.15 "
              "-p spk_model_path:=$HOME/vosk_models/vosk-model-spk-0.4 "
              "-p profile_file:=$HOME/speaker_profiles.pkl "
              "-p calibration_duration:=6 "
              "-p sample_rate:=16000 "
              "-p recording_delay:=1.5"
        ],
        output="screen",
    )

    # 3) Speaker Recognition Node
    speaker_recognition_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr speaker_recognition_node "
              "--ros-args "
              "-p profile_file:=$HOME/speaker_profiles.pkl "
              "-p threshold:=0.3"
        ],
        output="screen",
    )

    # 4) Calibration Workflow Node
    calibration_workflow_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr calibration_workflow"
        ],
        output="screen",
    )

    # 5) Command router
    cmd_router = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr ai_command_router "
              "--ros-args "
              "-p asr_topic:=/asr/text "
              "-p llm_request_topic:=/llm/request "
              "-p llm_response_topic:=/llm/response "
              "-p awake_topic:=/maki/awake"
        ],
        output="screen",
    )

    # 6) LLM bridge
    llm_bridge = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "cd ~/makimate-2026/src/core/server_llm/server_llm && "
              "python llm_bridge_node.py "
              "--ros-args "
              "-p laptop_host:='http://Swift3x.local:8000' "
              "-p endpoint_path:='/chat/stream' "
              "-p request_topic:=/llm/request "
              "-p stream_topic:=/llm/stream "
              "-p response_topic:=/llm/response "
              "-p asr_enable_topic:=/asr/enable "
              "--log-level asr_led_controller:=error"
        ],
        output="screen",
    )

    # 7) TTS
    tts_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr natural_tts_node "
              "--ros-args "
              "-p backend:=piper_python "
              "-p piper_command:=$HOME/makimate-2026/piper_bin/piper/piper "
              "-p piper_model:=$HOME/makimate-2026/piper_models/en_US-john-medium.onnx "
              "-p input_topic:=/llm/stream"
        ],
        output="screen",
    )

    # 8) LED ring
    led_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr asr_led_node "
              "--ros-args "
              "-p enable_topic:=/asr/enable "
              "-p awake_topic:=/maki/awake "
              "--log-level asr_led_controller:=error"
        ],
        output="screen",
    )

    # 9) Maki expressions
    maki_expressions = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_dxl maki_expressions"
        ],
        output="screen",
    )

    # 10) Awake → expression bridge
    maki_behavior_awake = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run maki_operational_nodes maki_awake_behavior "
              "--ros-args "
              "-p awake_topic:=/maki/awake "
              "-p expression_topic:=/maki/expression"
        ],
        output="screen",
    )

    # 11) DXL controller
    maki_dxl_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_dxl maki_dxl_6"
        ],
        output="screen",
    )

    # 12) Higher-level behavior
    maki_behavior_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_dxl maki_behavior "
              "--ros-args "
              "-p enable_monologue:=false"
        ],
        output="screen",
    )

    # 13) ReSpeaker DSP
    respeaker_dsp_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_asr respeaker_dsp_node "
              "--ros-args "
              "-p tuning_script_path:=$HOME/usb_4_mic_array/tuning.py "
              "-p poll_rate_hz:=10.0 "
              "-p vad_threshold:=3.5 "
              "-p enable_ns:=true "
              "-p enable_echo:=true "
              "-p enable_agc:=true "
              "-p hpf_cutoff:=2"
        ],
        output="screen",
    )

    # 14) Camera driver
    camera_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run camera_ros camera_node "
              "--ros-args "
              "-p camera:=0 "
              "-p role:=video "
              "-p sensor_mode:='640:480' "
              "-p width:=640 "
              "-p height:=480 "
              "-p format:=BGR888"
        ],
        output="screen",
    )

    # 15) Face tracker
    face_tracker_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_vision face_tracker"
        ],
        output="screen",
    )

    # 16) Face → Maki head control
    face_to_maki_node = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            shell_prefix
            + "ros2 run makimate_vision face_to_maki"
        ],
        output="screen",
    )

    return LaunchDescription([
        whisper_asr_node,       # Whisper: /asr/text
        vosk_embeddings_node,   # Vosk: /voice/speaker_embedding only
        voice_calibration_node,
        speaker_recognition_node,
        calibration_workflow_node,
        cmd_router,
        llm_bridge,
        tts_node,
        led_node,
        maki_expressions,
        maki_behavior_awake,
        maki_dxl_node,
        maki_behavior_node,
        respeaker_dsp_node,
        camera_node,
        face_tracker_node,
        face_to_maki_node,
    ])

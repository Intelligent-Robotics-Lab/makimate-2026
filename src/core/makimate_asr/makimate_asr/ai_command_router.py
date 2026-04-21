import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool, Float32

from makimate_interfaces.msg import FaceTrackArray

# Match speaker_recognition_node's own threshold — it already filters below this
VOICE_THRESHOLD = 0.30
# Minimum fused confidence to inject the user's name into the LLM message
FUSED_THRESHOLD = 0.30
# If a speaker ID arrived within this many seconds, skip the identity wait entirely
IDENTITY_FRESH_SECS = 2.0
# How long to wait for a speaker embedding if we don't have a fresh one
IDENTITY_WAIT_SECS = 0.30

# Phonetic mis-transcriptions of "Maki" that Whisper commonly produces.
# These are replaced with "maki" before any wake-phrase or LLM logic runs.
# Add more as you discover them in the logs.
_MAKI_ALIASES = [
    "mock", "mockey", "mocky", "makey", "mackey", "mackie",
    "marky", "markee", "markey", "maki mate", "mockey mate", "rocky", "Rocky",
]

# Word-level corrections for common Whisper mis-transcriptions.
# Applied before LLM forwarding. Format: (wrong, correct) — case-insensitive match.
_WORD_CORRECTIONS = [
    ("sector", "center"),
    ("louis", "louie"),
]

# ---------------------------------------------------------------------------
# Canned responses for the demo presentation.
# Each entry is (triggers, response) where triggers is a list of strings.
# If ANY trigger string appears anywhere in the utterance (case-insensitive),
# the canned response is spoken directly and the LLM is skipped.
# More specific triggers (longer phrases) should come first.
# ---------------------------------------------------------------------------
_CANNED_RESPONSES = [
    # Add demo Q&A pairs here. Examples:
    # (["thermodynamics"], "The thermodynamics lab is located on the third floor, room 312."),
    # (["advising", "advisor"], "Student advising offices are on the second floor near the main stairwell."),
]


class ASRCommandRouter(Node):
    """
    Extended to support multiple sleep phrases and speaker recognition.
    """

    def __init__(self):
        super().__init__("ai_command_router")

        # ---- Parameters ----
        self.declare_parameter("asr_topic", "/asr/text")
        self.declare_parameter("llm_request_topic", "/llm/request")
        self.declare_parameter("llm_response_topic", "/llm/response")
        self.declare_parameter("awake_topic", "/maki/awake")
        self.declare_parameter("tts_topic", "/llm/stream")
        self.declare_parameter("asr_enable_topic", "/asr/enable")
        self.declare_parameter("wake_phrase", "hello")
        self.declare_parameter("sleep_phrase", "good bye")
        self.declare_parameter("wake_greeting", "Hello! I'm awake and ready to talk. My name is Maki Mate, how may I help you.")
        self.declare_parameter("sleep_farewell", "Goodbye! I'm going back to sleep now.")
        self.declare_parameter("reply_keyword", "")  # if set, only forward to LLM when keyword present
        self.declare_parameter("identity_enabled", True)  # if False, skip speaker ID wait and name injection

        asr_topic = self.get_parameter("asr_topic").value
        llm_request_topic = self.get_parameter("llm_request_topic").value
        awake_topic = self.get_parameter("awake_topic").value
        tts_topic = self.get_parameter("tts_topic").value
        asr_enable_topic = self.get_parameter("asr_enable_topic").value

        self._wake_phrase = self.get_parameter("wake_phrase").value.lower()
        self._sleep_phrase = self.get_parameter("sleep_phrase").value.lower()
        self._wake_greeting = self.get_parameter("wake_greeting").value
        self._sleep_farewell = self.get_parameter("sleep_farewell").value
        self._reply_keyword = self.get_parameter("reply_keyword").value.lower().strip()

        self._sleep_phrases = [
            self._sleep_phrase, "goodbye", "good night", "goodnight",
            "night night", "good night maki", "goodnight maki", "good my", "dubai",
        ]
        self._reset_command = "/reset"

        # ---- Publishers ----
        _latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._llm_req_pub = self.create_publisher(String, llm_request_topic, 10)
        self._awake_pub = self.create_publisher(Bool, awake_topic, _latched_qos)
        self._tts_pub = self.create_publisher(String, tts_topic, 10)

        # ---- Identity state ----
        self.current_speaker = "Unknown"
        self.current_speaker_conf = 0.0
        self.current_face_name = "Unknown"
        self._last_speaker_id_time = 0.0  # epoch time of most recent speaker ID

        # Reentrant so identity callbacks can fire during sleep() in _on_asr
        _cg = ReentrantCallbackGroup()

        # ---- Subscribers ----
        self._asr_sub = self.create_subscription(String, asr_topic, self._on_asr, 10, callback_group=_cg)
        self._asr_enable_sub = self.create_subscription(Bool, asr_enable_topic, self._on_asr_enable, 10, callback_group=_cg)
        self.speaker_sub = self.create_subscription(
            String, '/voice/identified_speaker', self._on_speaker_identified, 10, callback_group=_cg)
        self.speaker_conf_sub = self.create_subscription(
            Float32, '/voice/speaker_confidence', self._on_speaker_confidence, 10, callback_group=_cg)
        self.face_tracks_sub = self.create_subscription(
            FaceTrackArray, '/maki/face_tracks', self._on_face_tracks, 10, callback_group=_cg)
        self._calibration_active = False
        self.create_subscription(Bool, '/calibration/active', self._on_calibration_active, 10, callback_group=_cg)

        # ---- State ----
        self._awake = False
        self._pending_sleep = False
        self._identity_enabled = bool(self.get_parameter("identity_enabled").value)

        self.add_on_set_parameters_callback(self._on_params)

        self._publish_awake(False)
        self.get_logger().info(f"AICommandRouter started. identity_enabled={self._identity_enabled}")

    # ------------------------------------------------------------------ #
    # Live parameter updates
    # ------------------------------------------------------------------ #
    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'identity_enabled':
                self._identity_enabled = bool(p.value)
                self.get_logger().info(f'[live] identity_enabled = {self._identity_enabled}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def _publish_awake(self, value: bool):
        msg = Bool()
        msg.data = value
        self._awake_pub.publish(msg)
        self._awake = value
        self.get_logger().info(f"Published awake={value}")

    def _speak_immediate(self, text: str):
        if not text:
            return
        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)
        self.get_logger().info(f"[Router->TTS] {text!r}")

    def _send_llm_command(self, command: str):
        if not command:
            return
        msg = String()
        msg.data = command
        self._llm_req_pub.publish(msg)
        self.get_logger().info(f"[Router->LLM] Sent command: {command!r}")

    # ------------------------------------------------------------------ #
    # Identity fusion
    # ------------------------------------------------------------------ #
    def _on_speaker_identified(self, msg: String):
        self.current_speaker = msg.data
        self._last_speaker_id_time = time.time()
        self.get_logger().info(f'Speaker updated to: {self.current_speaker}')

    def _on_speaker_confidence(self, msg: Float32):
        self.current_speaker_conf = float(msg.data)

    def _on_face_tracks(self, msg: FaceTrackArray):
        """Track the name of the best-scoring face from face_recognition."""
        if not msg.faces:
            self.current_face_name = "Unknown"
            return
        # face_tracker publishes tracks sorted by score descending; first = best
        best = msg.faces[0]
        self.current_face_name = best.name if best.name else "Unknown"

    def _fuse_identity(self):
        """
        Fuse voice and face identity signals.

        Returns (name: str | None, fused_confidence: float).

        Fusion rules:
          - Both modalities agree on same name → high confidence (0.4 + voice_conf)
          - Voice only (above threshold)       → voice_conf
          - Face only (recognition matched)    → 0.5 (moderate, no distance score)
          - Disagreement                        → (None, 0.0)
        """
        voice_name = self.current_speaker if self.current_speaker != "Unknown" else None
        voice_conf = self.current_speaker_conf
        face_name = self.current_face_name if self.current_face_name != "Unknown" else None

        # Discard weak voice ID
        if voice_name and voice_conf < VOICE_THRESHOLD:
            voice_name = None

        if voice_name and face_name:
            if voice_name.lower() == face_name.lower():
                fused_conf = min(1.0, 0.4 + voice_conf)
                self.get_logger().info(
                    f"Identity fusion: voice={voice_name}({voice_conf:.2f}) + "
                    f"face={face_name} → AGREE fused_conf={fused_conf:.2f}"
                )
                return voice_name, fused_conf
            else:
                self.get_logger().info(
                    f"Identity fusion: voice={voice_name} vs face={face_name} → DISAGREE"
                )
                return None, 0.0
        elif voice_name:
            self.get_logger().info(
                f"Identity fusion: voice only → {voice_name}({voice_conf:.2f})"
            )
            return voice_name, voice_conf
        elif face_name:
            self.get_logger().info(
                f"Identity fusion: face only → {face_name}(0.50)"
            )
            return face_name, 0.5
        else:
            return None, 0.0

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _on_asr(self, msg: String):
        text = msg.data.strip()
        low = text.lower()

        if not text:
            return

        self._turn_start_time = time.time()
        self.get_logger().info(f"[LATENCY] Router received ASR at t={self._turn_start_time:.3f}")
        self.get_logger().info(f"[ASR] Heard: {text!r}")

        # Normalise phonetic mis-transcriptions of "Maki" → "maki"
        _corrected = low
        for alias in _MAKI_ALIASES:
            _corrected = re.sub(r'\b' + re.escape(alias) + r'\b', 'maki', _corrected)
        if _corrected != low:
            self.get_logger().info(f"[ASR] Corrected to: {_corrected!r}")
            text = _corrected  # use corrected text for all further processing
            low  = _corrected

        # Apply general word corrections
        for wrong, right in _WORD_CORRECTIONS:
            _corrected = re.sub(r'\b' + re.escape(wrong) + r'\b', right, low, flags=re.IGNORECASE)
            if _corrected != low:
                self.get_logger().info(f"[ASR] Word correction: {wrong!r} → {right!r}")
                text = _corrected
                low  = _corrected

        # Drop Whisper hallucinations: repetitive phrases where one word
        # makes up >60% of all words (e.g. "I'm sorry I'm sorry I'm sorry...")
        words = low.split()
        if len(words) >= 6:
            most_common_count = max(words.count(w) for w in set(words))
            if most_common_count / len(words) > 0.6:
                self.get_logger().warn(
                    f"Dropping likely Whisper hallucination: {text[:80]!r}"
                )
                return

        self.get_logger().info(
            f"AICommandRouter received: {text!r} "
            f"(awake={self._awake}, pending_sleep={self._pending_sleep})"
        )

        # While asleep: only react to wake phrase
        if not self._awake:
            if self._wake_phrase in low:
                self._pending_sleep = False
                self._publish_awake(True)
                self._speak_immediate(self._wake_greeting)
            return

        # Calibration command — let calibration_workflow_node handle it exclusively
        if 'calibrate' in low or 'calibration' in low:
            self.get_logger().info("[Router] Calibration command — not forwarding to LLM.")
            return

        # Sleep phrases — go to sleep
        if any(phrase in low for phrase in self._sleep_phrases):
            self.get_logger().info(f"Sleep phrase detected: {text!r}")
            self._speak_immediate(self._sleep_farewell)
            self._pending_sleep = True
            self._send_llm_command(self._reset_command)
            return

        # Handle "hi/hello/hey" greetings — wait briefly for any fresher embedding
        if any(word in low.split() for word in ['hi', 'hello', 'hey']):
            self.get_logger().info("Greeting detected, waiting for speaker recognition...")
            time.sleep(0.30)
            name, conf = self._fuse_identity()
            response = f"Hi {name}!" if name and conf >= FUSED_THRESHOLD else "Hi there!"
            self.get_logger().info(f"Greeting response: {response}")
            self._speak_immediate(response)
            return

        # Keyword filter — if set, only reply when keyword is present in the utterance
        self._reply_keyword = self.get_parameter("reply_keyword").value.lower().strip()
        if self._reply_keyword and self._reply_keyword not in low:
            self.get_logger().info(
                f"Reply keyword '{self._reply_keyword}' not in utterance — ignoring."
            )
            return

        # Don't forward to LLM while calibration workflow is active
        if self._calibration_active:
            self.get_logger().info("[Router] Calibration active — not forwarding to LLM.")
            return

        # Canned response intercept — check before forwarding to LLM
        for triggers, canned in _CANNED_RESPONSES:
            if any(t in low for t in triggers):
                self.get_logger().info(
                    f"[Router] Canned response triggered by {triggers!r}: {canned!r}"
                )
                self._speak_immediate(canned)
                return

        # Normal conversation → forward to LLM with identity context.
        if not self._identity_enabled:
            llm_text = text
            self.get_logger().info("[LATENCY] Identity disabled — skipping wait and name injection.")
        else:
            # Only wait for a speaker embedding if we don't already have a fresh one.
            t_before_sleep = time.time()
            age = t_before_sleep - self._last_speaker_id_time
            if age > IDENTITY_FRESH_SECS:
                time.sleep(IDENTITY_WAIT_SECS)
            t_after_sleep = time.time()
            self.get_logger().info(
                f"[LATENCY] Identity wait: {(t_after_sleep - t_before_sleep)*1000:.0f}ms "
                f"(speaker age={age:.2f}s, router→LLM so far: {(t_after_sleep - self._turn_start_time)*1000:.0f}ms)"
            )

            name, conf = self._fuse_identity()
            if name and conf >= FUSED_THRESHOLD:
                llm_text = f"{text} (My name is {name}, please use it in your reply.)"
                self.get_logger().info(
                    f"Injecting identity into LLM: {name} (fused_conf={conf:.2f})"
                )
            else:
                llm_text = text
                if name:
                    self.get_logger().info(
                        f"Identity below threshold ({name}, conf={conf:.2f}) — not injecting"
                    )

        out = String()
        out.data = llm_text
        self._llm_req_pub.publish(out)
        self.get_logger().info(
            f"[LATENCY] Router→LLM forwarded at t={time.time():.3f} "
            f"(+{(time.time() - self._turn_start_time)*1000:.0f}ms from ASR received)"
        )

    def _on_calibration_active(self, msg: Bool):
        self._calibration_active = bool(msg.data)
        self.get_logger().info(f"Calibration active: {self._calibration_active}")

    def _on_asr_enable(self, msg: Bool):
        enabled = bool(msg.data)
        if self._pending_sleep and enabled:
            self._pending_sleep = False
            self._publish_awake(False)
            self.get_logger().info(
                "ASR re-enabled after goodbye → Maki going to sleep (LLM already reset)."
            )


def main(args=None):
    rclpy.init(args=args)
    node = ASRCommandRouter()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

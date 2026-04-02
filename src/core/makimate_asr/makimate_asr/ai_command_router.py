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

        # ---- State ----
        self._awake = False
        self._pending_sleep = False

        self._publish_awake(False)
        self.get_logger().info("AICommandRouter started.")

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

        # Normal conversation → forward to LLM with identity context.
        # Wait briefly for any fresher speaker embedding to arrive.
        time.sleep(0.30)

        name, conf = self._fuse_identity()
        if name and conf >= FUSED_THRESHOLD:
            llm_text = f"[The person speaking with you is {name}.] {text}"
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
        self.get_logger().info("[Router->LLM] forwarded user text.")

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

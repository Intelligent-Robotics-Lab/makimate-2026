import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time


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

        asr_topic = self.get_parameter("asr_topic").value
        llm_request_topic = self.get_parameter("llm_request_topic").value
        awake_topic = self.get_parameter("awake_topic").value
        tts_topic = self.get_parameter("tts_topic").value
        asr_enable_topic = self.get_parameter("asr_enable_topic").value

        self._wake_phrase = self.get_parameter("wake_phrase").value.lower()
        self._sleep_phrase = self.get_parameter("sleep_phrase").value.lower()
        self._wake_greeting = self.get_parameter("wake_greeting").value
        self._sleep_farewell = self.get_parameter("sleep_farewell").value

        self._sleep_phrases = [
            self._sleep_phrase, "goodbye", "good night", "goodnight",
            "night night", "good night maki", "goodnight maki", "good my", "dubai",
        ]
        self._reset_command = "/reset"

        # ---- Publishers ----
        self._llm_req_pub = self.create_publisher(String, llm_request_topic, 10)
        self._awake_pub = self.create_publisher(Bool, awake_topic, 10)
        self._tts_pub = self.create_publisher(String, tts_topic, 10)

        # Speaker tracking
        self.current_speaker = "Unknown"

        # ---- Subscribers ----
        self._asr_sub = self.create_subscription(String, asr_topic, self._on_asr, 10)
        self._asr_enable_sub = self.create_subscription(Bool, asr_enable_topic, self._on_asr_enable, 10)
        self.speaker_sub = self.create_subscription(String, '/voice/identified_speaker', self._on_speaker_identified, 10)

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
        """Send a control command like /reset to the LLM via /llm/request."""
        if not command:
            return
        msg = String()
        msg.data = command
        self._llm_req_pub.publish(msg)
        self.get_logger().info(f"[Router->LLM] Sent command: {command!r}")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _on_speaker_identified(self, msg):
        """Track who is currently speaking."""
        self.current_speaker = msg.data
        self.get_logger().info(f'Speaker updated to: {self.current_speaker}')
    
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
            
        # Handle "hi" greetings when awake
        if any(word in low.split() for word in ['hi', 'hello', 'hey']):
            self.get_logger().info("Greeting detected, waiting for speaker recognition...")
            
            # Wait for the speaker identification message with timeout
            try:
                from rclpy import wait_for_message
                speaker_msg = wait_for_message(
                    String,
                    self,
                    '/voice/identified_speaker',
                    timeout=0.5
                )
                speaker = speaker_msg.data
                self.get_logger().info(f"Got speaker from wait_for_message: {speaker}")
            except Exception as e:
                self.get_logger().warn(f"No speaker message received: {e}")
                speaker = "Unknown"
            
            # Generate greeting
            if speaker and speaker != "Unknown":
                response = f"Hi {speaker}!"
            else:
                response = f"Hi there!"
            
            self.get_logger().info(f"Greeting response: {response}")
            self._speak_immediate(response)
            return
            
        # Check sleep phrases
        if any(phrase in low for phrase in self._sleep_phrases):
            self._speak_immediate(self._sleep_farewell)
            self._send_llm_command(self._reset_command)
            self._pending_sleep = True
            self.get_logger().info(
                f"Sleep phrase detected in: {text!r}. "
                "Sent /reset to LLM and waiting for ASR re-enable before sleeping."
            )
            return

        # Normal conversation: forward to LLM
        out = String()
        out.data = text
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

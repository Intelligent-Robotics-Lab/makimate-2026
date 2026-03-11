#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import re


class CalibrationWorkflowNode(Node):
    """
    Conversational calibration workflow.
    Asks questions, collects natural speech samples.
    """

    def __init__(self):
        super().__init__('calibration_workflow_node')

        # Publishers
        self.tts_pub = self.create_publisher(String, '/llm/stream', 10)
        self.start_cal_pub = self.create_publisher(String, '/voice/start_calibration', 10)
        self.record_pub = self.create_publisher(Bool, '/voice/record_segment', 10)
        self.finish_pub = self.create_publisher(Bool, '/voice/finish_calibration', 10)

        # Subscribers
        self.create_subscription(String, '/asr/text', self._on_asr_text, 10)
        self.create_subscription(Float32, '/voice/calibration_progress', self._on_progress, 10)
        self.create_subscription(Bool, '/voice/calibration_ready', self._on_ready, 10)
        self.create_subscription(Bool, '/asr/enable', self._on_asr_enable, 10)
        self.create_subscription(String, '/voice/calibration_status', self._on_status, 10)

        # State
        self.state = "IDLE"  # IDLE, WAITING_FOR_NAME, ASKING_QUESTIONS
        self.pending_name = None
        self.current_question_idx = 0
        self.samples_collected = 0
        self.target_samples = 5
        self.recording = False

        # Questions for natural conversation
        self.questions = [
            "What's your favorite color?",
            "Where did you grow up?",
            "What do you like to do for fun?",
            "What's your favorite food?",
            "Do you have any pets?",
            "What's your favorite movie?",
            "What kind of music do you like?",
            "What's your dream vacation spot?",
            "What do you do for work or study?",
            "Tell me about your hobbies.",
        ]

        self.get_logger().info("Calibration Workflow Node ready")
        self.get_logger().info("Say 'calibrate' to begin voice enrollment")

    def _speak(self, text: str):
        """Send text to TTS."""
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)

    def _extract_name(self, text: str) -> str:
        """Extract name from speech."""
        text = text.strip()
        
        patterns = [
            r"my name is (\w+)",
            r"i'?m (\w+)",
            r"call me (\w+)",
            r"it'?s (\w+)",
            r"this is (\w+)",
            r"(\w+)",
        ]
        
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                name = match.group(1).capitalize()
                
                stopwords = {
                    'the', 'a', 'an', 'and', 'or', 'but', 'is', 'was',
                    'what', 'when', 'where', 'who', 'how', 'why',
                    'yes', 'no', 'okay', 'ok', 'please', 'thank', 'thanks'
                }
                
                if name.lower() not in stopwords and len(name) > 1:
                    return name
        
        return None

    def _on_asr_text(self, msg: String):
        """Handle incoming ASR text."""
        text = msg.data.strip().lower()

        if self.state == "IDLE":
            if "calibrate" in text:
                self.get_logger().info("Calibration requested!")
                self.state = "WAITING_FOR_NAME"
                self._speak("What's your name?")
                self.get_logger().info("Maki: \"What's your name?\"")

        elif self.state == "WAITING_FOR_NAME":
            # Capture the name
            name = self._extract_name(msg.data)
            
            if name:
                self.pending_name = name
                self.get_logger().info(f"Name captured: {name}")

                # Start calibration session
                start_msg = String()
                start_msg.data = name
                self.start_cal_pub.publish(start_msg)

                # Move to question asking
                self.state = "ASKING_QUESTIONS"
                self.current_question_idx = 0
                self.samples_collected = 0

                # First instruction
                self._speak(f"Okay {name}, I'm going to ask you a few questions. Just answer naturally.")
                self.get_logger().info(f"Maki: \"Okay {name}, I'm going to ask you a few questions.\"")
            else:
                self._speak("I didn't catch that. What's your name?")

        elif self.state == "ASKING_QUESTIONS":
            # User is answering a question - this triggers recording
            if self.recording:
                self.get_logger().info(f"User answered: {text}")

    def _on_ready(self, msg: Bool):
        """Voice calibration node signals it's ready for next recording."""
        if not msg.data or self.state != "ASKING_QUESTIONS":
            return

        # Ask next question if we haven't collected enough samples
        if self.samples_collected < self.target_samples and self.current_question_idx < len(self.questions):
            question = self.questions[self.current_question_idx]
            self._speak(question)
            self.get_logger().info(f"Maki: \"{question}\"")
            self.current_question_idx += 1

    def _on_asr_enable(self, msg: Bool):
        """When ASR re-enables after TTS, start recording."""
        if not msg.data or self.state != "ASKING_QUESTIONS":
            return

        # TTS finished, now start recording the user's answer
        self.get_logger().info("ASR enabled - starting to record answer")
        self.recording = True
        
        record_msg = Bool()
        record_msg.data = True
        self.record_pub.publish(record_msg)

    def _on_progress(self, msg: Float32):
        """Track calibration progress."""
        if self.state != "ASKING_QUESTIONS":
            return

        progress = int(msg.data * 100)
        new_samples = int(msg.data * self.target_samples)

        if new_samples > self.samples_collected:
            self.samples_collected = new_samples
            self.get_logger().info(f"Progress: {progress}% ({self.samples_collected}/{self.target_samples} samples)")

            # Stop recording this segment
            self.recording = False
            record_msg = Bool()
            record_msg.data = False
            self.record_pub.publish(record_msg)

            # Check if done
            if self.samples_collected >= self.target_samples:
                self.get_logger().info("Enough samples collected, finishing calibration")
                finish_msg = Bool()
                finish_msg.data = True
                self.finish_pub.publish(finish_msg)
                
                # Don't speak here - wait for calibration_status callback

    def _on_status(self, msg: String):
        """Handle calibration completion status."""
        status = msg.data
        
        if status.startswith("success:") and self.state == "ASKING_QUESTIONS":
            self._speak(f"Great! I've learned your voice, {self.pending_name}.")
            self.get_logger().info(f"Maki: \"Great! I've learned your voice, {self.pending_name}.\"")
            
            # Reset state
            self.state = "IDLE"
            self.pending_name = None
            self.current_question_idx = 0
            self.samples_collected = 0
            self.recording = False


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationWorkflowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Conversational Voice Calibration Workflow for MakiMate
1. User says "calibrate" (or "cowboy", "campaign" - common misrecognitions)
2. Maki: "What's your name?"
3. User: "John"
4. Maki: "Okay John, I'm going to ask you a few questions. Just answer naturally."
5. Maki asks questions, user answers each one
6. After 5 samples: "Great! I've learned your voice, John."
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import re


class CalibrationWorkflowNode(Node):
    """Manages conversational voice calibration flow with multiple questions."""

    def __init__(self):
        super().__init__('calibration_workflow_node')

        # State machine
        self.state = 'IDLE'  # IDLE, WAITING_FOR_NAME, ASKING_QUESTIONS
        self.pending_name = None
        self.current_question_idx = 0
        self.samples_collected = 0
        self.target_samples = 5
        self.recording = False

        # Publishers
        self.tts_pub = self.create_publisher(String, '/llm/stream', 10)
        self.calibration_start_pub = self.create_publisher(String, '/voice/start_calibration', 10)
        self.record_segment_pub = self.create_publisher(Bool, '/voice/record_segment', 10)
        self.finish_calibration_pub = self.create_publisher(Bool, '/voice/finish_calibration', 10)

        # Subscribers
        self.asr_sub = self.create_subscription(
            String,
            '/asr/text',
            self.asr_callback,
            10
        )

        self.calibration_status_sub = self.create_subscription(
            String,
            '/voice/calibration_status',
            self.calibration_status_callback,
            10
        )

        self.calibration_progress_sub = self.create_subscription(
            Float32,
            '/voice/calibration_progress',
            self.calibration_progress_callback,
            10
        )

        self.calibration_ready_sub = self.create_subscription(
            Bool,
            '/voice/calibration_ready',
            self.calibration_ready_callback,
            10
        )

        self.asr_enable_sub = self.create_subscription(
            Bool,
            '/asr/enable',
            self.asr_enable_callback,
            10
        )

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

        self.get_logger().info('Calibration Workflow Node ready (conversational mode)')
        self.get_logger().info('Say "calibrate" to begin voice enrollment')

    def speak(self, text):
        """Send text to TTS."""
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f'Maki: "{text}"')

    def asr_callback(self, msg):
        """Handle speech recognition results."""
        text = msg.data.lower().strip()

        if not text:
            return

        self.get_logger().debug(f'ASR [{self.state}]: "{text}"')

        if self.state == 'IDLE':
            # Lenient matching for "calibrate" - common misrecognitions in noisy environments
            if any(word in text for word in ['calibrate', 'calibration', 'cowboy', 'campaign', 'calendar']):
                self.get_logger().info('Calibration requested!')
                self.state = 'WAITING_FOR_NAME'
                self.speak("What's your name?")

        elif self.state == 'WAITING_FOR_NAME':
            name = self.extract_name(text)

            if name:
                self.pending_name = name
                self.get_logger().info(f'Name captured: {name}')

                # Start calibration session
                cal_msg = String()
                cal_msg.data = name
                self.calibration_start_pub.publish(cal_msg)

                self.state = 'ASKING_QUESTIONS'
                self.current_question_idx = 0
                self.samples_collected = 0
                self.recording = False

                self.speak(f"Okay {name}, I'm going to ask you a few questions. Just answer naturally.")

            else:
                self.speak("I didn't catch that. What's your name?")

        elif self.state == 'ASKING_QUESTIONS':
            # User is answering - logged for debugging
            if self.recording:
                self.get_logger().info(f"User answered: {text}")

    def extract_name(self, text):
        """Extract a name from speech."""
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

    def calibration_ready_callback(self, msg):
        """Calibration node signals it's ready - ask first question."""
        if not msg.data or self.state != 'ASKING_QUESTIONS':
            return

        # Ask next question if we haven't collected enough samples
        if self.samples_collected < self.target_samples and self.current_question_idx < len(self.questions):
            question = self.questions[self.current_question_idx]
            self.speak(question)
            self.current_question_idx += 1

    def asr_enable_callback(self, msg):
        """When ASR re-enables after TTS, start recording the answer."""
        if not msg.data or self.state != 'ASKING_QUESTIONS':
            return

        # TTS finished asking question, now record the user's answer
        self.get_logger().info("ASR enabled - starting to record answer")
        self.recording = True

        start_msg = Bool()
        start_msg.data = True
        self.record_segment_pub.publish(start_msg)

    def calibration_progress_callback(self, msg):
        """Track progress as samples are collected."""
        if self.state != 'ASKING_QUESTIONS':
            return
    
        progress = msg.data
        new_samples = int(progress * self.target_samples)
    
        if new_samples > self.samples_collected:
            self.samples_collected = new_samples
            self.get_logger().info(f'Progress: {int(progress * 100)}% ({self.samples_collected}/{self.target_samples} samples)')
    
            # Stop recording this segment
            self.recording = False
            stop_msg = Bool()
            stop_msg.data = False
            self.record_segment_pub.publish(stop_msg)
    
            # Check if done
            if self.samples_collected >= self.target_samples:
                self.get_logger().info('Enough samples collected, finishing calibration')
                finish_msg = Bool()
                finish_msg.data = True
                self.finish_calibration_pub.publish(finish_msg)
            else:
                # Ask next question directly (don't wait for ready callback)
                if self.current_question_idx < len(self.questions):
                    question = self.questions[self.current_question_idx]
                    self.speak(question)
                    self.current_question_idx += 1

    def calibration_status_callback(self, msg):
        """Handle calibration completion status."""
        status = msg.data

        self.get_logger().info(f'Calibration status: {status}')

        if status.startswith('success:'):
            parts = status.split(':')
            if len(parts) >= 2:
                name = parts[1]
                self.speak(f"Great! I've learned your voice, {name}.")
            else:
                self.speak("Great! I've learned your voice.")

            # Reset state
            self.state = 'IDLE'
            self.pending_name = None
            self.current_question_idx = 0
            self.samples_collected = 0
            self.recording = False

        elif status.startswith('error:'):
            error_type = status.split(':')[1] if ':' in status else 'unknown'

            if error_type == 'no_audio':
                self.speak("I didn't hear anything. Please try again and speak clearly.")
            elif error_type == 'no_speaker_data':
                self.speak("I couldn't capture your voice properly. Please try again.")
            elif error_type == 'busy':
                self.speak("Calibration is already in progress. Please wait.")
            else:
                self.speak("Sorry, something went wrong. Please try again later.")

            # Reset state
            self.state = 'IDLE'
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

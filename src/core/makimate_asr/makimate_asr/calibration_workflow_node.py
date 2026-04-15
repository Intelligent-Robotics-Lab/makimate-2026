#!/usr/bin/env python3
"""
Voice Calibration Workflow Node for MakiMate
Handles conversational calibration workflow:
1. User says "calibrate"
2. Maki: "What's your name?"
3. User: "John"
4. Maki: "Okay John, please speak for 10 seconds starting now"
5. [Records 10 seconds]
6. Maki: "All done! Your voice is calibrated."
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import re


class CalibrationWorkflowNode(Node):
    """Manages the voice calibration conversation flow."""

    def __init__(self):
        super().__init__('calibration_workflow_node')

        # State machine
        self.state = 'IDLE'
        self.pending_name = None

        # Publishers
        self.tts_pub = self.create_publisher(String, '/llm/stream', 10)
        self.calibration_start_pub = self.create_publisher(String, '/voice/start_calibration', 10)
        self.record_segment_pub = self.create_publisher(Bool, '/voice/record_segment', 10)
        self.finish_calibration_pub = self.create_publisher(Bool, '/voice/finish_calibration', 10)
        self.calibration_active_pub = self.create_publisher(Bool, '/calibration/active', 10)

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

        self.get_logger().info('Calibration Workflow Node ready')
        self.get_logger().info('Say "calibrate" to begin voice enrollment')

    def _set_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.calibration_active_pub.publish(msg)

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
            if 'calibrate' in text or 'calibration' in text:
                self.get_logger().info('Calibration requested!')
                self.state = 'WAITING_FOR_NAME'
                self._set_active(True)
                self.speak("What's your name?")

        elif self.state == 'WAITING_FOR_NAME':
            name = self.extract_name(text)

            if name:
                self.pending_name = name
                self.get_logger().info(f'Name captured: {name}')

                self.speak(f"Okay {name}, please speak for 10 seconds starting now.")

                self.state = 'CALIBRATING'

                cal_msg = String()
                cal_msg.data = name
                self.calibration_start_pub.publish(cal_msg)

            else:
                self.speak("I didn't catch that. What's your name?")

        elif self.state == 'CALIBRATING':
            pass

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
        """Start recording once calibration node is ready."""
        if not msg.data or self.state != 'CALIBRATING':
            return

        self.get_logger().info('Calibration ready — starting recording')

        start_msg = Bool()
        start_msg.data = True
        self.record_segment_pub.publish(start_msg)

        self.get_logger().info('🎤 Recording for 10 seconds...')

        # Timer to stop recording
        self.record_timer = self.create_timer(10.0, self.stop_recording)

    def stop_recording(self):
        """Stop recording and finish calibration."""
        self.record_timer.cancel()

        stop_msg = Bool()
        stop_msg.data = False
        self.record_segment_pub.publish(stop_msg)

        self.get_logger().info('⏹️ Recording finished')

        finish_msg = Bool()
        finish_msg.data = True
        self.finish_calibration_pub.publish(finish_msg)

    def calibration_status_callback(self, msg):
        """Handle calibration status updates."""
        status = msg.data

        self.get_logger().info(f'Calibration status: {status}')

        if status.startswith('success:'):
            parts = status.split(':')
            if len(parts) >= 2:
                name = parts[1]
                self.speak(f"All done! Your voice is calibrated, {name}.")
            else:
                self.speak("All done! Your voice is calibrated.")

            self.state = 'IDLE'
            self.pending_name = None
            self._set_active(False)

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

            self.state = 'IDLE'
            self.pending_name = None
            self._set_active(False)

    def calibration_progress_callback(self, msg):
        """Handle calibration progress updates."""
        progress = msg.data

        if self.state == 'CALIBRATING':
            if 40 <= progress < 45:
                self.get_logger().info('Calibration 40% complete')
            elif 80 <= progress < 85:
                self.get_logger().info('Calibration 80% complete - almost done!')


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

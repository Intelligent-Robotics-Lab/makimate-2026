#!/usr/bin/env python3
"""
Real-Time Display Logger for MakiMate
Shows clean, formatted logs of speaker recognition and system events.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from datetime import datetime
import sys


class RealtimeDisplayLogger(Node):
    """Clean real-time display of MakiMate system events."""
    
    def __init__(self):
        super().__init__('realtime_display_logger')
        
        # State tracking
        self.current_speaker = "Unknown"
        self.speaker_confidence = 0.0
        self.is_awake = False
        self.calibration_active = False
        
        # Subscribers
        self.create_subscription(String, '/asr/text', self.asr_callback, 10)
        self.create_subscription(String, '/voice/identified_speaker', self.speaker_callback, 10)
        self.create_subscription(Float32, '/voice/speaker_confidence', self.confidence_callback, 10)
        self.create_subscription(Bool, '/maki/awake', self.awake_callback, 10)
        self.create_subscription(String, '/voice/calibration_status', self.calibration_status_callback, 10)
        self.create_subscription(Float32, '/voice/calibration_progress', self.calibration_progress_callback, 10)
        self.create_subscription(String, '/llm/stream', self.llm_stream_callback, 10)
        self.create_subscription(String, '/llm/response', self.llm_response_callback, 10)
        
        self.print_header()
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)  # Suppress default logs
    
    def print_header(self):
        """Print clean header."""
        print("\n" + "="*80)
        print(" "*25 + "MAKIMATE REAL-TIME LOG")
        print("="*80)
        print(f"{'TIME':<12} {'SPEAKER':<15} {'CONF':<6} {'EVENT':<10} {'MESSAGE':<35}")
        print("-"*80)
        sys.stdout.flush()
    
    def timestamp(self):
        """Get formatted timestamp."""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    def log(self, event_type, message, speaker=None, confidence=None):
        """Print formatted log line."""
        spk = speaker if speaker else self.current_speaker
        conf = f"{confidence:.2f}" if confidence is not None else f"{self.speaker_confidence:.2f}"
        
        # Color codes (optional - remove if terminal doesn't support)
        if event_type == "SPEECH":
            prefix = "🎤"
        elif event_type == "MAKI":
            prefix = "🤖"
        elif event_type == "CALIBRATE":
            prefix = "📝"
        elif event_type == "AWAKE":
            prefix = "👁️"
        elif event_type == "SLEEP":
            prefix = "😴"
        elif event_type == "SPEAKER":
            prefix = "👤"
        else:
            prefix = "ℹ️"
        
        print(f"{self.timestamp():<12} {spk:<15} {conf:<6} {prefix} {event_type:<8} {message:<35}")
        sys.stdout.flush()
    
    def asr_callback(self, msg):
        """Handle ASR transcription."""
        text = msg.data.strip()
        if text:
            self.log("SPEECH", text)
    
    def speaker_callback(self, msg):
        """Handle speaker identification."""
        new_speaker = msg.data
        if new_speaker != self.current_speaker:
            self.current_speaker = new_speaker
            self.log("SPEAKER", f"Now speaking: {new_speaker}")
    
    def confidence_callback(self, msg):
        """Handle speaker confidence updates."""
        self.speaker_confidence = msg.data
    
    def awake_callback(self, msg):
        """Handle wake/sleep state."""
        was_awake = self.is_awake
        self.is_awake = msg.data
        
        if self.is_awake and not was_awake:
            self.log("AWAKE", "Maki is now awake and listening")
        elif not self.is_awake and was_awake:
            self.log("SLEEP", "Maki is now sleeping")
    
    def calibration_status_callback(self, msg):
        """Handle calibration status."""
        status = msg.data
        
        if status.startswith("started:"):
            name = status.split(":")[1] if ":" in status else "Unknown"
            self.calibration_active = True
            self.log("CALIBRATE", f"Calibration started for {name}")
        
        elif status.startswith("success:"):
            parts = status.split(":")
            name = parts[1] if len(parts) > 1 else "Unknown"
            samples = parts[2] if len(parts) > 2 else "?"
            self.calibration_active = False
            self.log("CALIBRATE", f"✓ {name} calibrated ({samples} samples)")
        
        elif status.startswith("error:"):
            error = status.split(":")[1] if ":" in status else "unknown"
            self.calibration_active = False
            self.log("CALIBRATE", f"✗ Calibration failed: {error}")
    
    def calibration_progress_callback(self, msg):
        """Handle calibration progress."""
        if self.calibration_active:
            progress = msg.data
            if progress % 20 == 0:  # Log every 20%
                self.log("CALIBRATE", f"Recording... {progress:.0f}% complete")
    
    def llm_stream_callback(self, msg):
        """Handle LLM streaming response."""
        # TTS input - this is what Maki will say
        text = msg.data.strip()
        if text and len(text) > 10:  # Only log substantial responses
            self.log("MAKI", text[:45])  # Truncate long responses
    
    def llm_response_callback(self, msg):
        """Handle complete LLM response."""
        pass  # We're using stream instead


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeDisplayLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" + "="*80)
        print(" "*30 + "LOG ENDED")
        print("="*80 + "\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyaudio
import queue
import threading
import numpy as np
from faster_whisper import WhisperModel


class ReSpeakerWhisperASR(Node):
    def __init__(self):
        super().__init__('respeaker_whisper_asr')
        
        # Parameters
        self.declare_parameter('sample_rate', 16000.0)
        self.declare_parameter('device', 1)
        self.declare_parameter('model_size', 'base')  # tiny, base, small, medium, large
        self.declare_parameter('asr_topic', '/asr/text')
        self.declare_parameter('enable_topic', '/asr/enable')
        
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.device_index = self.get_parameter('device').value
        model_size = self.get_parameter('model_size').value
        asr_topic = self.get_parameter('asr_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        
        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = WhisperModel(model_size, device="cpu", compute_type="int8")
        
        # Publishers
        self.text_pub = self.create_publisher(String, asr_topic, 10)
        
        # Subscribers
        self.create_subscription(Bool, enable_topic, self._on_asr_enable, 10)
        
        # State
        self.enabled = True
        self.audio_q = queue.Queue()
        self.audio_buffer = []
        
        # Start audio stream
        self.get_logger().info('Initializing audio stream...')
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=8000,
            stream_callback=self._audio_callback
        )
        self.stream.start_stream()
        
        # Start processing thread
        self.asr_thread = threading.Thread(target=self._asr_loop, daemon=True)
        self.asr_thread.start()
        
        self.get_logger().info(f'Audio stream started at {self.sample_rate} Hz, device={self.device_index}')
        self.get_logger().info('ReSpeakerWhisperASR node started.')
    
    def _audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudio callback - captures audio."""
        if self.enabled:
            self.audio_buffer.append(in_data)
        return (in_data, pyaudio.paContinue)
    
    def _asr_loop(self):
        """Process audio buffer every 2 seconds."""
        self.get_logger().info("ASR processing thread running.")
        import time
        
        while rclpy.ok():
            time.sleep(2.0)  # Process every 2 seconds
            
            if not self.enabled or len(self.audio_buffer) == 0:
                continue
            
            # Get audio data
            audio_data = b''.join(self.audio_buffer)
            self.audio_buffer.clear()
            
            # Convert to float32 numpy array
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
            
            try:
                # Run Whisper inference
                segments, info = self.model.transcribe(
                    audio_np,
                    language="en",
                    vad_filter=True,
                    vad_parameters=dict(min_silence_duration_ms=500)
                )
                
                # Collect text
                text_parts = []
                for segment in segments:
                    text_parts.append(segment.text.strip())
                
                if text_parts:
                    full_text = ' '.join(text_parts)
                    if full_text:
                        self.get_logger().info(f'ASR text: {full_text!r}')
                        msg = String()
                        msg.data = full_text
                        self.text_pub.publish(msg)
            
            except Exception as e:
                self.get_logger().error(f'Whisper error: {e}')
    
    def _on_asr_enable(self, msg):
        """Handle ASR enable/disable."""
        enabled = bool(msg.data)
        if enabled != self.enabled:
            self.enabled = enabled
            state = "enabled" if enabled else "disabled"
            self.get_logger().info(f'ASR {state}: {"will resume" if enabled else "flushing input queue"}.')
            if not enabled:
                self.audio_buffer.clear()
    
    def destroy_node(self):
        self.get_logger().info("Shutting down ReSpeakerWhisperASR node...")
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.p:
            self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReSpeakerWhisperASR()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

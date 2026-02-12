#!/usr/bin/env python3
import os
import json
import pickle
import numpy as np
from vosk import Model, KaldiRecognizer, SpkModel
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import threading
import time


class SpeakerRecognitionNode(Node):
    """ROS2 node for real-time speaker recognition."""
    
    def __init__(self):
        super().__init__('speaker_recognition_node')
        
        # Parameters
        self.declare_parameter('vosk_model_path', '/home/emanuel/vosk_models/vosk-model-small-en-us-0.15')
        self.declare_parameter('spk_model_path', '/home/emanuel/vosk_models/vosk-model-spk-0.4')
        self.declare_parameter('profile_file', os.path.expanduser('~/speaker_profiles.pkl'))
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('threshold', 0.75)  # Similarity threshold
        self.declare_parameter('device_index', -1)  # -1 = default mic
        
        # Get parameters
        vosk_model = self.get_parameter('vosk_model_path').value
        spk_model = self.get_parameter('spk_model_path').value
        self.profile_file = self.get_parameter('profile_file').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.threshold = self.get_parameter('threshold').value
        self.device_index = self.get_parameter('device_index').value
        
        # Load models
        self.get_logger().info(f'Loading Vosk model: {vosk_model}')
        if not os.path.exists(vosk_model):
            self.get_logger().error(f'Vosk model not found: {vosk_model}')
            raise FileNotFoundError(f'Vosk model not found: {vosk_model}')
        
        self.model = Model(vosk_model)
        
        self.get_logger().info(f'Loading speaker model: {spk_model}')
        if not os.path.exists(spk_model):
            self.get_logger().error(f'Speaker model not found: {spk_model}')
            raise FileNotFoundError(f'Speaker model not found: {spk_model}')
        
        self.spk_model = SpkModel(spk_model)
        
        # Load speaker profiles
        self.speaker_profiles = {}
        self.load_profiles()
        
        # State
        self.is_running = False
        self.recognition_thread = None
        
        # Publishers
        self.speaker_pub = self.create_publisher(String, '/voice/identified_speaker', 10)
        self.confidence_pub = self.create_publisher(Float32, '/voice/speaker_confidence', 10)
        
        # Start recognition automatically
        self.start_recognition()
        
        self.get_logger().info('Speaker Recognition Node ready')
        self.get_logger().info(f'Loaded {len(self.speaker_profiles)} speaker profiles')
        self.get_logger().info(f'Recognition threshold: {self.threshold}')
    
    def load_profiles(self):
        """Load speaker profiles from disk."""
        if os.path.exists(self.profile_file):
            try:
                with open(self.profile_file, 'rb') as f:
                    self.speaker_profiles = pickle.load(f)
                
                # Convert lists back to numpy arrays
                for name in self.speaker_profiles:
                    if isinstance(self.speaker_profiles[name], list):
                        self.speaker_profiles[name] = np.array(self.speaker_profiles[name])
                
                self.get_logger().info(f'Loaded {len(self.speaker_profiles)} profiles from {self.profile_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load profiles: {e}')
        else:
            self.get_logger().warn('No speaker profiles found - run calibration first')
    
    def cosine_similarity(self, vec1, vec2):
        """Calculate cosine similarity between two vectors."""
        return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
    
    def identify_speaker(self, speaker_vector):
        """
        Identify a speaker from their voice vector.
        Returns (speaker_name, confidence) or (None, best_score)
        """
        if not self.speaker_profiles:
            return None, 0.0
        
        best_match = None
        best_score = 0.0
        
        for name, profile_vector in self.speaker_profiles.items():
            similarity = self.cosine_similarity(speaker_vector, profile_vector)
            if similarity > best_score:
                best_score = similarity
                best_match = name
        
        if best_score >= self.threshold:
            return best_match, best_score
        else:
            return None, best_score
    
    def start_recognition(self):
        """Start continuous speaker recognition."""
        if self.is_running:
            self.get_logger().warn('Recognition already running')
            return
        
        self.is_running = True
        self.recognition_thread = threading.Thread(
            target=self.recognition_loop,
            daemon=True
        )
        self.recognition_thread.start()
        self.get_logger().info('Started continuous speaker recognition')
    
    def stop_recognition(self):
        """Stop speaker recognition."""
        self.is_running = False
        if self.recognition_thread:
            self.recognition_thread.join(timeout=2.0)
        self.get_logger().info('Stopped speaker recognition')
    
    def recognition_loop(self):
        """Main recognition loop - runs continuously."""
        
        rec = KaldiRecognizer(self.model, self.sample_rate)
        rec.SetSpkModel(self.spk_model)
        rec.SetWords(True)
        
        p = pyaudio.PyAudio()
        
        # Select microphone device
        if self.device_index == -1:
            device = None  # Use default
        else:
            device = self.device_index
        
        try:
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=device,
                frames_per_buffer=8000
            )
            self.get_logger().info(f'Opened microphone (device: {device})')
        except Exception as e:
            self.get_logger().error(f'Failed to open microphone: {e}')
            p.terminate()
            self.is_running = False
            return
        
        stream.start_stream()
        
        current_speaker = None
        last_publish_time = 0
        publish_interval = 0.5  # Publish at most every 0.5 seconds
        
        self.get_logger().info('Listening for speakers...')
        
        try:
            while self.is_running and rclpy.ok():
                try:
                    data = stream.read(4000, exception_on_overflow=False)
                    
                    if rec.AcceptWaveform(data):
                        result = json.loads(rec.Result())
                        
                        # Check if we have speaker data
                        if 'spk' in result:
                            speaker_vector = np.array(result['spk'])
                            speaker, confidence = self.identify_speaker(speaker_vector)
                            
                            now = time.time()
                            
                            # Only publish if speaker changed or enough time passed
                            if (speaker != current_speaker) or (now - last_publish_time > publish_interval):
                                # Publish identified speaker
                                speaker_msg = String()
                                speaker_msg.data = speaker if speaker else "Unknown"
                                self.speaker_pub.publish(speaker_msg)
                                
                                # Publish confidence
                                conf_msg = Float32()
                                conf_msg.data = confidence
                                self.confidence_pub.publish(conf_msg)
                                
                                if speaker != current_speaker:
                                    if speaker:
                                        self.get_logger().info(f'Speaker: {speaker} (confidence: {confidence:.2f})')
                                    else:
                                        self.get_logger().info(f'Unknown speaker (best match: {confidence:.2f})')
                                
                                current_speaker = speaker
                                last_publish_time = now
                        
                        # Log transcription
                        if 'text' in result and result['text']:
                            self.get_logger().debug(f'[{current_speaker or "Unknown"}] {result["text"]}')
                
                except Exception as e:
                    self.get_logger().warn(f'Recognition error: {e}')
                    continue
        
        except KeyboardInterrupt:
            pass
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()
            self.get_logger().info('Recognition loop ended')


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recognition()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

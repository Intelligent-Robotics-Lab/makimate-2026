#!/usr/bin/env python3
import os
import json
import pickle
import numpy as np
from vosk import Model, KaldiRecognizer, SpkModel
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
import threading


class VoiceCalibrationNode(Node):
    """ROS2 node for speaker voice calibration/enrollment."""
    
    def __init__(self):
        super().__init__('voice_calibration_node')
        
        # Parameters
        self.declare_parameter('vosk_model_path', '/home/emanuel/vosk_models/vosk-model-small-en-us-0.15')
        self.declare_parameter('spk_model_path', '/home/emanuel/vosk_models/vosk-model-spk-0.4')
        self.declare_parameter('profile_file', os.path.expanduser('~/speaker_profiles.pkl'))
        self.declare_parameter('calibration_duration', 10)  # seconds
        self.declare_parameter('sample_rate', 16000)
        
        # Get parameters
        vosk_model = self.get_parameter('vosk_model_path').value
        spk_model = self.get_parameter('spk_model_path').value
        self.profile_file = self.get_parameter('profile_file').value
        self.calibration_duration = self.get_parameter('calibration_duration').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
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
        
        # Load existing profiles
        self.speaker_profiles = {}
        self.load_profiles()
        
        # State
        self.is_calibrating = False
        self.calibration_thread = None
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/voice/calibration_status', 10)
        self.progress_pub = self.create_publisher(Float32, '/voice/calibration_progress', 10)
        
        # Subscribers
        self.start_sub = self.create_subscription(
            String,
            '/voice/start_calibration',
            self.start_calibration_callback,
            10
        )
        
        self.get_logger().info('Voice Calibration Node ready')
        self.get_logger().info(f'Loaded {len(self.speaker_profiles)} speaker profiles')
        self.get_logger().info('Publish speaker name to /voice/start_calibration to begin')
    
    def load_profiles(self):
        """Load speaker profiles from disk."""
        if os.path.exists(self.profile_file):
            try:
                with open(self.profile_file, 'rb') as f:
                    self.speaker_profiles = pickle.load(f)
                self.get_logger().info(f'Loaded {len(self.speaker_profiles)} profiles from {self.profile_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load profiles: {e}')
        else:
            self.get_logger().info('No existing profile file found - starting fresh')
    
    def save_profiles(self):
        """Save speaker profiles to disk."""
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(self.profile_file), exist_ok=True)
            
            with open(self.profile_file, 'wb') as f:
                pickle.dump(self.speaker_profiles, f)
            self.get_logger().info(f'Saved {len(self.speaker_profiles)} profiles to {self.profile_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save profiles: {e}')
    
    def start_calibration_callback(self, msg):
        """Start calibration for a speaker."""
        speaker_name = msg.data.strip()
        
        if not speaker_name:
            self.get_logger().warn('Empty speaker name received')
            status_msg = String()
            status_msg.data = 'error:empty_name'
            self.status_pub.publish(status_msg)
            return
        
        if self.is_calibrating:
            self.get_logger().warn('Calibration already in progress')
            status_msg = String()
            status_msg.data = f'error:busy'
            self.status_pub.publish(status_msg)
            return
        
        self.get_logger().info(f'Starting calibration for: {speaker_name}')
        
        # Publish status
        status_msg = String()
        status_msg.data = f'started:{speaker_name}'
        self.status_pub.publish(status_msg)
        
        # Start calibration in thread
        self.calibration_thread = threading.Thread(
            target=self.calibrate_speaker,
            args=(speaker_name,),
            daemon=True
        )
        self.calibration_thread.start()
    
    def calibrate_speaker(self, speaker_name):
        """Perform speaker calibration from microphone."""
        self.is_calibrating = True
        
        try:
            rec = KaldiRecognizer(self.model, self.sample_rate)
            rec.SetSpkModel(self.spk_model)
            
            p = pyaudio.PyAudio()
            
            # Open microphone
            try:
                stream = p.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=self.sample_rate,
                    input=True,
                    frames_per_buffer=8000
                )
                self.get_logger().info('Microphone opened')
            except Exception as e:
                self.get_logger().error(f'Failed to open microphone: {e}')
                status_msg = String()
                status_msg.data = f'error:microphone:{e}'
                self.status_pub.publish(status_msg)
                p.terminate()
                self.is_calibrating = False
                return
            
            stream.start_stream()
            
            speaker_vectors = []
            transcriptions = []
            frames = 0
            max_frames = self.calibration_duration * self.sample_rate // 4000
            audio_detected = False
            
            self.get_logger().info(f'Recording for {self.calibration_duration} seconds...')
            
            try:
                while frames < max_frames and self.is_calibrating:
                    try:
                        data = stream.read(4000, exception_on_overflow=False)
                        
                        # Check audio level
                        audio_data = np.frombuffer(data, dtype=np.int16)
                        volume = np.abs(audio_data).mean()
                        
                        if volume > 100:
                            audio_detected = True
                        
                        # Publish progress
                        progress = (frames / max_frames) * 100.0
                        progress_msg = Float32()
                        progress_msg.data = progress
                        self.progress_pub.publish(progress_msg)
                        
                        if rec.AcceptWaveform(data):
                            result = json.loads(rec.Result())
                            
                            if 'text' in result and result['text']:
                                transcriptions.append(result['text'])
                                self.get_logger().info(f'Heard: "{result["text"]}"')
                            
                            if 'spk' in result:
                                speaker_vectors.append(result['spk'])
                                self.get_logger().info(f'Captured sample {len(speaker_vectors)}')
                        
                        frames += 1
                        
                    except Exception as e:
                        self.get_logger().warn(f'Read error: {e}')
                        continue
                
                # Process final audio
                final_result = json.loads(rec.FinalResult())
                if 'text' in final_result and final_result['text']:
                    transcriptions.append(final_result['text'])
                if 'spk' in final_result:
                    speaker_vectors.append(final_result['spk'])
                
            finally:
                stream.stop_stream()
                stream.close()
                p.terminate()
            
            # Validate results
            if not audio_detected:
                self.get_logger().error('No audio detected!')
                status_msg = String()
                status_msg.data = 'error:no_audio'
                self.status_pub.publish(status_msg)
                self.is_calibrating = False
                return
            
            if not speaker_vectors:
                self.get_logger().error('No speaker data captured!')
                status_msg = String()
                status_msg.data = 'error:no_speaker_data'
                self.status_pub.publish(status_msg)
                self.is_calibrating = False
                return
            
            # Average the vectors
            avg_vector = np.mean(speaker_vectors, axis=0)
            self.speaker_profiles[speaker_name] = avg_vector.tolist()  # Convert to list for JSON serialization
            self.save_profiles()
            
            self.get_logger().info(f'✅ Calibrated "{speaker_name}" with {len(speaker_vectors)} samples')
            
            # Publish success
            status_msg = String()
            status_msg.data = f'success:{speaker_name}:{len(speaker_vectors)}'
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
            status_msg = String()
            status_msg.data = f'error:exception:{e}'
            self.status_pub.publish(status_msg)
        
        finally:
            self.is_calibrating = False
            
            # Publish 100% progress
            progress_msg = Float32()
            progress_msg.data = 100.0
            self.progress_pub.publish(progress_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

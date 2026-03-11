#!/usr/bin/env python3
import os
import pickle
import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32


class VoiceCalibrationNode(Node):
    """ROS2 node for speaker voice calibration using embeddings from main ASR."""
    
    def __init__(self):
        super().__init__('voice_calibration_node')
        
        # Parameters
        self.declare_parameter('profile_file', os.path.expanduser('~/speaker_profiles.pkl'))
        self.declare_parameter('target_samples', 5)
        
        self.profile_file = self.get_parameter('profile_file').value
        self.target_samples = self.get_parameter('target_samples').value
        
        # Load existing profiles
        self.speaker_profiles = {}
        self.load_profiles()
        
        # Calibration state
        self.is_calibrating = False
        self.current_speaker_name = None
        self.speaker_vectors = []
        self.recording_active = False
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/voice/calibration_status', 10)
        self.progress_pub = self.create_publisher(Float32, '/voice/calibration_progress', 10)
        self.ready_pub = self.create_publisher(Bool, '/voice/calibration_ready', 10)
        
        # Subscribers
        self.start_sub = self.create_subscription(
            String, '/voice/start_calibration', self.start_calibration_callback, 10
        )
        
        self.record_segment_sub = self.create_subscription(
            Bool, '/voice/record_segment', self.record_segment_callback, 10
        )
        
        self.finish_calibration_sub = self.create_subscription(
            Bool, '/voice/finish_calibration', self.finish_calibration_callback, 10
        )
        
        # Subscribe to embeddings from main ASR node
        self.embedding_sub = self.create_subscription(
            String, '/voice/speaker_embedding', self.embedding_callback, 10
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
            os.makedirs(os.path.dirname(self.profile_file), exist_ok=True)
            with open(self.profile_file, 'wb') as f:
                pickle.dump(self.speaker_profiles, f)
            self.get_logger().info(f'Saved {len(self.speaker_profiles)} profiles to {self.profile_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save profiles: {e}')
    
    def start_calibration_callback(self, msg):
        """Start calibration session for a speaker."""
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
            status_msg.data = 'error:busy'
            self.status_pub.publish(status_msg)
            return
        
        self.get_logger().info(f'Starting calibration session for: {speaker_name}')
        
        self.is_calibrating = True
        self.current_speaker_name = speaker_name
        self.speaker_vectors = []
        self.recording_active = False
        
        # Publish status
        status_msg = String()
        status_msg.data = f'started:{speaker_name}'
        self.status_pub.publish(status_msg)
        
        # Signal ready for first question
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_pub.publish(ready_msg)
    
    def record_segment_callback(self, msg):
        """Start/stop recording a segment."""
        if not self.is_calibrating:
            return
        
        should_record = msg.data
        
        if should_record and not self.recording_active:
            self.recording_active = True
            self.get_logger().info('🎤 Recording segment started')
            
        elif not should_record and self.recording_active:
            self.recording_active = False
            self.get_logger().info('⏹️  Recording segment stopped')
            
            # Signal ready for next question
            ready_msg = Bool()
            ready_msg.data = True
            self.ready_pub.publish(ready_msg)
    
    def embedding_callback(self, msg):
        """Receive speaker embedding from main ASR node."""
        if not self.is_calibrating or not self.recording_active:
            return
        
        try:
            data = json.loads(msg.data)
            if 'spk' in data:
                self.speaker_vectors.append(data['spk'])
                self.get_logger().info(f'✅ Captured sample {len(self.speaker_vectors)}/{self.target_samples}')
                
                # Publish progress
                progress = len(self.speaker_vectors) / self.target_samples
                progress_msg = Float32()
                progress_msg.data = progress
                self.progress_pub.publish(progress_msg)
                
                # Stop recording this segment automatically
                self.recording_active = False
                
        except Exception as e:
            self.get_logger().error(f'Error processing embedding: {e}')
    
    def finish_calibration_callback(self, msg):
        """Finish calibration and save profile."""
        if not self.is_calibrating:
            return
        
        self.recording_active = False
        
        if len(self.speaker_vectors) < self.target_samples:
            self.get_logger().error(
                f'Insufficient samples: {len(self.speaker_vectors)} (need {self.target_samples})'
            )
            status_msg = String()
            status_msg.data = 'error:insufficient_samples'
            self.status_pub.publish(status_msg)
            self.cleanup_calibration()
            return
        
        try:
            # Average speaker vectors
            avg_vector = np.mean(self.speaker_vectors, axis=0)
            self.speaker_profiles[self.current_speaker_name] = avg_vector.tolist()
            self.save_profiles()
            
            self.get_logger().info(
                f'✅ Calibrated "{self.current_speaker_name}" with {len(self.speaker_vectors)} samples'
            )
            
            status_msg = String()
            status_msg.data = f'success:{self.current_speaker_name}:{len(self.speaker_vectors)}'
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
            status_msg = String()
            status_msg.data = f'error:save_failed:{e}'
            self.status_pub.publish(status_msg)
        
        finally:
            self.cleanup_calibration()
    
    def cleanup_calibration(self):
        """Clean up after calibration."""
        self.is_calibrating = False
        self.current_speaker_name = None
        self.speaker_vectors = []
        self.recording_active = False
        self.get_logger().info('Calibration session ended')
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.cleanup_calibration()
        super().destroy_node()


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

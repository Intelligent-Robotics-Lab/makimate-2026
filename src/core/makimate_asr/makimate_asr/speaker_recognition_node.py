#!/usr/bin/env python3
import os
import json
import pickle
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class SpeakerRecognitionNode(Node):
    """ROS2 node for real-time speaker recognition from embeddings."""
    
    def __init__(self):
        super().__init__('speaker_recognition_node')
        
        # Parameters
        self.declare_parameter('profile_file', os.path.expanduser('~/speaker_profiles.pkl'))
        self.declare_parameter('threshold', 0.75)  # Similarity threshold
        
        # Get parameters
        self.profile_file = self.get_parameter('profile_file').value
        self.threshold = self.get_parameter('threshold').value
        
        # Load speaker profiles
        self.speaker_profiles = {}
        self.load_profiles()
        
        # State
        self.current_speaker = None
        
        # Publishers
        self.speaker_pub = self.create_publisher(String, '/voice/identified_speaker', 10)
        self.confidence_pub = self.create_publisher(Float32, '/voice/speaker_confidence', 10)
        
        # Subscriber to speaker embeddings from ASR
        self.embedding_sub = self.create_subscription(
            String,
            '/voice/speaker_embedding',
            self.embedding_callback,
            10
        )
        
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
            self.get_logger().warn('No speaker profiles loaded!') #NEW: self.get_logger
            return None, 0.0

        self.get_logger().info(f'Comparing against {len(self.speaker_profiles)} profiles') #NEW: self.get_logger
        
        best_match = None
        best_score = 0.0
        
        for name, profile_vector in self.speaker_profiles.items():
            similarity = self.cosine_similarity(speaker_vector, profile_vector)
            self.get_logger().info(f'  {name}: similarity={similarity:.4f}') #NEW: self.get_logger
            if similarity > best_score:
                best_score = similarity
                best_match = name

        self.get_logger().info(f'Best match: {best_match} with score {best_score:.4f} (threshold: {self.threshold})') #NEW: self.get_logger
        
        if best_score >= self.threshold:
            return best_match, best_score
        else:
            return None, best_score
    
    def embedding_callback(self, msg):
        """Process incoming speaker embeddings from ASR."""
        self.get_logger().info(f'Received embedding message (length: {len(msg.data)})')
        
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f'Parsed JSON successfully, keys: {data.keys()}')
            
            if 'spk' not in data:
                self.get_logger().warn('No "spk" key in embedding data')
                return
            
            speaker_vector = np.array(data['spk'])
            self.get_logger().info(f'Speaker vector length: {len(speaker_vector)}')
            
            speaker, confidence = self.identify_speaker(speaker_vector)
            
            # Always log for debugging
            self.get_logger().info(f'Recognition: speaker={speaker}, confidence={confidence:.3f}, threshold={self.threshold}')
            
            ##### Only publish if speaker changed
            # NEW: ALWAYS publish speaker (not just on change)
            speaker_msg = String()
            speaker_msg.data = speaker if speaker else "Unknown"
            self.speaker_pub.publish(speaker_msg)
            
            # Publish confidence
            conf_msg = Float32()
            conf_msg.data = confidence
            self.confidence_pub.publish(conf_msg)
            
            # Log if speaker changed
            if speaker != self.current_speaker:
                if speaker:
                    self.get_logger().info(f'Speaker: {speaker} (confidence: {confidence:.2f})')
                else:
                    self.get_logger().info(f'Unknown speaker (best match: {confidence:.2f})')
                self.current_speaker = speaker
            """
            if speaker != self.current_speaker:
                # Publish identified speaker
                speaker_msg = String()
                speaker_msg.data = speaker if speaker else "Unknown"
                self.speaker_pub.publish(speaker_msg)
                
                # Publish confidence
                conf_msg = Float32()
                conf_msg.data = confidence
                self.confidence_pub.publish(conf_msg)
                
                if speaker:
                    self.get_logger().info(f'Speaker: {speaker} (confidence: {confidence:.2f})')
                else:
                    self.get_logger().info(f'Unknown speaker (best match: {confidence:.2f})')
                
                self.current_speaker = speaker
            """
        
        except Exception as e:
            self.get_logger().error(f'Error processing embedding: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

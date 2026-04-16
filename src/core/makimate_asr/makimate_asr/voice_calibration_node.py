#!/usr/bin/env python3
import os
import json
import pickle
import numpy as np
from vosk import Model, KaldiRecognizer, SpkModel
import sounddevice as sd
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
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('target_samples', 1)  # Number of voice samples to collect
        
        # Get parameters
        vosk_model = self.get_parameter('vosk_model_path').value
        spk_model = self.get_parameter('spk_model_path').value
        self.profile_file = self.get_parameter('profile_file').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.target_samples = self.get_parameter('target_samples').value
        
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
        
        # Calibration state
        self.is_calibrating = False
        self.current_speaker_name = None
        self.speaker_vectors = []
        self.asr_enabled = False
        self.recording_active = False
        self.recording_thread = None
        
        # Audio setup
        self.stream = None
        self.rec = None
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/voice/calibration_status', 10)
        self.progress_pub = self.create_publisher(Float32, '/voice/calibration_progress', 10)
        self.ready_pub = self.create_publisher(Bool, '/voice/calibration_ready', 10)
        self.asr_enable_pub = self.create_publisher(Bool, '/asr/enable', 10)
        
        # Subscribers
        self.start_sub = self.create_subscription(
            String,
            '/voice/start_calibration',
            self.start_calibration_callback,
            10
        )
        
        self.asr_enable_sub = self.create_subscription(
            Bool,
            '/asr/enable',
            self.asr_enable_callback,
            10
        )
        
        self.record_segment_sub = self.create_subscription(
            Bool,
            '/voice/record_segment',
            self.record_segment_callback,
            10
        )
        
        self.finish_calibration_sub = self.create_subscription(
            Bool,
            '/voice/finish_calibration',
            self.finish_calibration_callback,
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
        
        # Initialize recognizer but DON'T open audio stream yet
        self.rec = KaldiRecognizer(self.model, self.sample_rate)
        self.rec.SetSpkModel(self.spk_model)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'started:{speaker_name}'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('Waiting for TTS to finish before opening microphone...')
    
    def asr_enable_callback(self, msg):
        """Track when ASR is enabled/disabled (TTS finished/started)."""
        self.asr_enabled = msg.data

        if self.is_calibrating and self.asr_enabled and not self.recording_active:
            # TTS finished — signal ready. Stream opens in record_segment_callback
            # only after ASR is disabled so we get exclusive mic access.
            self.get_logger().info('TTS finished - ready to record')
            ready_msg = Bool()
            ready_msg.data = True
            self.ready_pub.publish(ready_msg)
    
    def record_segment_callback(self, msg):
        """Start/stop recording a segment."""
        if not self.is_calibrating:
            return

        should_record = msg.data

        if should_record and not self.recording_active:
            # Disable ASR first so respeaker_whisper_asr releases the mic
            asr_msg = Bool()
            asr_msg.data = False
            self.asr_enable_pub.publish(asr_msg)
            self.get_logger().info('ASR disabled — waiting for mic release...')

            # Start in background so we can sleep without blocking the ROS executor
            threading.Thread(target=self._start_recording, daemon=True).start()
            
        elif not should_record and self.recording_active:
            # Stop recording
            self.recording_active = False
            self.get_logger().info('⏹️  Recording segment stopped')
            # Processing happens in _record_loop when it exits
    
    def _start_recording(self):
        """Open mic after a short delay (gives respeaker time to release ALSA device)."""
        import time
        time.sleep(0.5)

        try:
            self.stream = sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=4000,
                device='respeaker_shared',  # matches respeaker_vosk_asr
                dtype='int16',
                channels=1,
            )
            self.stream.start()
            self.get_logger().info('✅ Audio stream opened — recording started')
        except Exception as e:
            self.get_logger().error(f'Failed to open microphone: {e}')
            status_msg = String()
            status_msg.data = f'error:audio_init:{e}'
            self.status_pub.publish(status_msg)
            self.cleanup_calibration()
            return

        self.recording_active = True
        self.rec.Reset()
        self.get_logger().info('🎤 Recording segment started')
        self._record_loop()

    def _record_loop(self):
        """Background thread that reads audio while recording_active is True."""
        while self.recording_active and self.stream:
            try:
                data, _ = self.stream.read(4000)
                
                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    if 'spk' in result:
                        self.speaker_vectors.append(result['spk'])
                        self.get_logger().info(f'Captured speaker vector ({len(self.speaker_vectors)} so far)')
                    if 'text' in result and result['text']:
                        self.get_logger().debug(f'Heard: "{result["text"]}"')
            
            except Exception as e:
                self.get_logger().warn(f'Audio read error: {e}')
                break
        
        # Recording stopped, process the segment
        if self.is_calibrating:
            self._process_recorded_segment()
    
    def _process_recorded_segment(self):
        """Process the currently recorded segment and extract speaker vector."""
        try:
            # Get final result from recognizer
            final_result = json.loads(self.rec.FinalResult())
            
            if 'spk' in final_result:
                self.speaker_vectors.append(final_result['spk'])
                self.get_logger().info(f'✅ Captured sample {len(self.speaker_vectors)}/{self.target_samples}')
                
                # Publish progress
                progress = (len(self.speaker_vectors) / self.target_samples) * 100.0
                progress_msg = Float32()
                progress_msg.data = progress
                self.progress_pub.publish(progress_msg)
                
                # Transcription feedback (optional)
                if 'text' in final_result and final_result['text']:
                    self.get_logger().debug(f'Transcribed: "{final_result["text"]}"')
            else:
                self.get_logger().warn('No speaker data in this segment')
                
        except Exception as e:
            self.get_logger().error(f'Error processing segment: {e}')
        
    def finish_calibration_callback(self, msg):
        """Finish calibration and save profile."""
        if not self.is_calibrating:
            return
    
        # Stop recording if still active
        self.recording_active = False
    
        # Wait for recording thread to finish processing
        if self.recording_thread:
            self.recording_thread.join(timeout=2.0)
            self.recording_thread = None
    
        # Now check collected samples
        if len(self.speaker_vectors) < self.target_samples:
            self.get_logger().error(
                f'Insufficient samples: {len(self.speaker_vectors)} (need at least {self.target_samples})'
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
        """Clean up audio resources after calibration."""
        self.recording_active = False
        
        if self.recording_thread:
            self.recording_thread.join(timeout=1.0)
            self.recording_thread = None
        
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None
        
        self.is_calibrating = False
        self.current_speaker_name = None
        self.speaker_vectors = []

        # Re-enable ASR
        asr_msg = Bool()
        asr_msg.data = True
        self.asr_enable_pub.publish(asr_msg)
        self.get_logger().info('ASR re-enabled after calibration')

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

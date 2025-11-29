#!/usr/bin/env python3
"""
Voice Command Node - Offline Speech Recognition with Vosk

Listens to the Go2's built-in microphone and performs offline speech recognition
using the Vosk ASR engine. Publishes recognized target object keywords to the
mission executive FSM.

EDTH Hackathon - Oscar Devos

Subscriptions:
    /audiohub/data (unitree_go/AudioData)  - Raw audio from Go2 microphone
    /mission/state (std_msgs/String)       - Current FSM state

Publications:
    /mission/target_object (std_msgs/String) - Recognized target object keyword

The node uses a constrained grammar vocabulary to improve recognition accuracy
in the noisy robot environment. Audio is only processed when the mission
executive is in the LISTENING state.

For laptop testing, set use_pyaudio:=true to use local microphone instead of
ROS topic.
"""

import json
import os
import struct
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from unitree_go.msg import AudioData
    UNITREE_AVAILABLE = True
except ImportError:
    UNITREE_AVAILABLE = False

try:
    from vosk import Model, KaldiRecognizer
    VOSK_AVAILABLE = True
except ImportError:
    VOSK_AVAILABLE = False

try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
except ImportError:
    PYAUDIO_AVAILABLE = False


class VoiceCommandNode(Node):
    """
    Offline speech recognition node using Vosk ASR.
    
    Listens to the Go2's built-in microphone via ROS topic and performs
    keyword recognition with a constrained vocabulary. Only processes
    audio when the mission executive is in the LISTENING state.
    """

    def __init__(self):
        super().__init__('voice_command_node')
        
        # ============================================================
        # PARAMETERS
        # ============================================================
        self.declare_parameter('model_path', '/opt/vosk/vosk-model-small-en-us-0.15')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('audio_topic', '/audiohub/data')
        # Default vocabulary - must match COCO class names exactly!
        # Override with vosk_params.yaml for full vocabulary
        self.declare_parameter('vocabulary', [
            'person', 'backpack', 'handbag', 'suitcase', 'umbrella',
            'laptop', 'cell phone', 'remote', 'keyboard', 'mouse', 'tv',
            'bottle', 'cup', 'wine glass', 'book', 'chair', 'couch', 'clock',
            'vase', 'scissors', 'potted plant', 'toothbrush', 'hair dryer',
            'microwave', 'oven', 'refrigerator', 'teddy bear', 'sports ball',
            'cat', 'dog', 'bird'
        ])
        self.declare_parameter('input_sample_rate', 16000)  # Go2 mic sample rate
        self.declare_parameter('input_channels', 1)
        self.declare_parameter('input_sample_width', 2)  # 16-bit audio
        self.declare_parameter('use_pyaudio', False)  # Use local mic for laptop testing
        self.declare_parameter('pyaudio_device_index', -1)  # -1 = default device
        
        # Get parameters
        model_path_raw = self.get_parameter('model_path').value
        # Expand ~ and $HOME in model path
        self.model_path = os.path.expanduser(os.path.expandvars(model_path_raw))
        
        # Log parameter source for debugging
        self.get_logger().info(f'  Model path parameter (raw): {model_path_raw}')
        self.get_logger().info(f'  Model path (expanded): {self.model_path}')
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.audio_topic = self.get_parameter('audio_topic').value
        self.vocabulary = list(self.get_parameter('vocabulary').value)
        self.input_sample_rate = self.get_parameter('input_sample_rate').value
        self.input_channels = self.get_parameter('input_channels').value
        self.input_sample_width = self.get_parameter('input_sample_width').value
        self.use_pyaudio = self.get_parameter('use_pyaudio').value
        self.pyaudio_device_index = self.get_parameter('pyaudio_device_index').value
        
        # ============================================================
        # VOSK INITIALIZATION
        # ============================================================
        self.model: Optional[Model] = None
        self.recognizer: Optional[KaldiRecognizer] = None
        
        if not VOSK_AVAILABLE:
            self.get_logger().error(
                'Vosk library not available! Install with: pip install vosk'
            )
        else:
            self._init_vosk()
        
        # ============================================================
        # STATE TRACKING
        # ============================================================
        self.current_state = 'IDLE'
        self.is_listening = False
        self.audio_buffer = bytearray()
        self.pyaudio_stream = None
        self.pyaudio_instance = None
        self.audio_thread = None
        self.running = True
        
        # ============================================================
        # ROS SUBSCRIBERS
        # ============================================================
        self.state_sub = self.create_subscription(
            String,
            '/mission/state',
            self.state_callback,
            10
        )
        
        # Audio input: either ROS topic or PyAudio
        if self.use_pyaudio:
            self._init_pyaudio()
        else:
            if UNITREE_AVAILABLE:
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                self.audio_sub = self.create_subscription(
                    AudioData,
                    self.audio_topic,
                    self.audio_callback,
                    qos
                )
            else:
                self.get_logger().error(
                    'unitree_go messages not available! Use use_pyaudio:=true for laptop testing'
                )
        
        # ============================================================
        # ROS PUBLISHERS
        # ============================================================
        self.target_pub = self.create_publisher(
            String,
            '/mission/target_object',
            10
        )
        
        self.get_logger().info('Voice Command Node initialized')
        self.get_logger().info(f'  Model path: {self.model_path}')
        self.get_logger().info(f'  Sample rate: {self.sample_rate} Hz')
        if self.use_pyaudio:
            self.get_logger().info(f'  Audio source: PyAudio (local microphone)')
        else:
            self.get_logger().info(f'  Audio source: {self.audio_topic}')
        self.get_logger().info(f'  Vocabulary: {len(self.vocabulary)} words')
        
    def _init_vosk(self):
        """Initialize Vosk model and recognizer with grammar constraints."""
        try:
            self.get_logger().info(f'Loading Vosk model from {self.model_path}...')
            self.model = Model(self.model_path)
            
            # Create recognizer with grammar constraint
            # The grammar is a JSON array of valid words/phrases
            grammar = json.dumps(self.vocabulary)
            self.recognizer = KaldiRecognizer(self.model, self.sample_rate, grammar)
            
            # Enable word-level timestamps for better partial results
            self.recognizer.SetWords(True)
            
            self.get_logger().info('Vosk model loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load Vosk model: {e}')
            self.get_logger().error(
                'Download a model from https://alphacephei.com/vosk/models'
            )
            self.model = None
            self.recognizer = None
    
    def _init_pyaudio(self):
        """Initialize PyAudio for local microphone input (laptop testing)."""
        if not PYAUDIO_AVAILABLE:
            self.get_logger().warn(
                'PyAudio not available! Install with: pip install pyaudio. '
                'Audio processing will be disabled.'
            )
            return
        
        try:
            self.pyaudio_instance = pyaudio.PyAudio()
            
            # Check if any input devices are available
            input_devices = []
            for i in range(self.pyaudio_instance.get_device_count()):
                dev_info = self.pyaudio_instance.get_device_info_by_index(i)
                if dev_info['maxInputChannels'] > 0:
                    input_devices.append((i, dev_info['name']))
            
            if not input_devices:
                self.get_logger().warn(
                    'No audio input devices found. PyAudio disabled. '
                    'This is normal in Docker containers without audio access.'
                )
                self.pyaudio_instance.terminate()
                self.pyaudio_instance = None
                return
            
            # Get device info
            device_index = None if self.pyaudio_device_index < 0 else self.pyaudio_device_index
            
            if device_index is None:
                # Try to find default input device
                try:
                    default_info = self.pyaudio_instance.get_default_input_device_info()
                    device_index = default_info['index']
                    self.get_logger().info(f'Using default audio device: {default_info["name"]}')
                except OSError:
                    # No default device, use first available
                    if input_devices:
                        device_index = input_devices[0][0]
                        self.get_logger().info(f'Using audio device: {input_devices[0][1]}')
                    else:
                        raise
            
            self.pyaudio_stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=4096
            )
            
            # Start audio capture thread
            self.audio_thread = threading.Thread(target=self._pyaudio_capture_loop)
            self.audio_thread.daemon = True
            self.audio_thread.start()
            
            self.get_logger().info('PyAudio initialized - using local microphone')
            
        except OSError as e:
            # Common in Docker/headless environments
            self.get_logger().warn(
                f'PyAudio initialization failed (no audio device): {e}. '
                'Audio processing disabled. This is normal in Docker containers.'
            )
            if self.pyaudio_instance:
                try:
                    self.pyaudio_instance.terminate()
                except:
                    pass
            self.pyaudio_instance = None
            self.pyaudio_stream = None
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PyAudio: {e}')
            if self.pyaudio_instance:
                try:
                    self.pyaudio_instance.terminate()
                except:
                    pass
            self.pyaudio_instance = None
            self.pyaudio_stream = None
    
    def _pyaudio_capture_loop(self):
        """Background thread for capturing audio from local microphone."""
        while self.running and self.pyaudio_stream:
            try:
                audio_data = self.pyaudio_stream.read(4096, exception_on_overflow=False)
                self._process_audio_data(audio_data)
            except Exception as e:
                self.get_logger().error(f'PyAudio capture error: {e}')
                break
    
    def cleanup(self):
        """Clean up resources."""
        self.running = False
        
        if self.pyaudio_stream:
            self.pyaudio_stream.stop_stream()
            self.pyaudio_stream.close()
        
        if self.pyaudio_instance:
            self.pyaudio_instance.terminate()
        
        if self.audio_thread and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=1.0)
    
    def state_callback(self, msg: String):
        """
        Handle mission state updates.
        
        Only process audio when in LISTENING state. When entering LISTENING,
        reset the recognizer and audio buffer for a fresh recognition session.
        """
        new_state = msg.data
        
        if new_state != self.current_state:
            self.get_logger().debug(f'State changed: {self.current_state} -> {new_state}')
            
            # Entering LISTENING state
            if new_state == 'LISTENING':
                self.is_listening = True
                self.audio_buffer.clear()
                
                # Reset recognizer for fresh recognition
                if self.recognizer:
                    self.recognizer.Reset()
                    
                self.get_logger().info('Entering LISTENING mode - speak target object name')
                
            # Leaving LISTENING state
            elif self.current_state == 'LISTENING':
                self.is_listening = False
                self.get_logger().info('Exiting LISTENING mode')
                
            self.current_state = new_state
    
    def audio_callback(self, msg):
        """
        Process incoming audio data from Go2 microphone (ROS topic).
        
        Audio is only processed when in LISTENING state. The raw audio bytes
        are passed to the Vosk recognizer which handles buffering internally.
        When a final result is available, the recognized keyword is published.
        """
        # Get raw audio bytes from message
        audio_data = bytes(msg.data)
        self._process_audio_data(audio_data)
    
    def _process_audio_data(self, audio_data: bytes):
        """
        Process raw audio data from any source (ROS topic or PyAudio).
        """
        if not self.is_listening:
            return
            
        if self.recognizer is None:
            return
        
        if len(audio_data) == 0:
            return
        
        # Resample if input sample rate differs from Vosk expected rate
        if self.input_sample_rate != self.sample_rate and not self.use_pyaudio:
            audio_data = self._resample_audio(audio_data)
        
        # Feed audio to recognizer
        # AcceptWaveform returns True when it has a final result
        if self.recognizer.AcceptWaveform(audio_data):
            result = json.loads(self.recognizer.Result())
            self._process_result(result, is_final=True)
        else:
            # Check partial results for early feedback
            partial = json.loads(self.recognizer.PartialResult())
            self._process_result(partial, is_final=False)
    
    def _resample_audio(self, audio_data: bytes) -> bytes:
        """
        Resample audio from input sample rate to Vosk expected rate.
        
        Uses simple linear interpolation for resampling. For better quality,
        consider using scipy.signal.resample or librosa.
        """
        # Convert bytes to numpy array (16-bit signed integers)
        samples = np.frombuffer(audio_data, dtype=np.int16)
        
        # Calculate resampling ratio
        ratio = self.sample_rate / self.input_sample_rate
        new_length = int(len(samples) * ratio)
        
        # Simple linear interpolation resampling
        indices = np.linspace(0, len(samples) - 1, new_length)
        resampled = np.interp(indices, np.arange(len(samples)), samples)
        
        # Convert back to bytes
        return resampled.astype(np.int16).tobytes()
    
    def _process_result(self, result: dict, is_final: bool):
        """
        Process recognition result and publish if valid keyword found.
        
        For final results, extracts the 'text' field and checks if it
        matches any word in the vocabulary. For partial results, only
        logs for debugging purposes.
        """
        if is_final:
            text = result.get('text', '').strip().lower()
            
            if not text:
                self.get_logger().debug('Empty recognition result')
                return
            
            self.get_logger().info(f'Recognized: "{text}"')
            
            # Check if recognized text matches vocabulary
            # Handle multi-word phrases and single words
            matched_keyword = None
            
            for keyword in self.vocabulary:
                keyword_lower = keyword.lower()
                if keyword_lower in text or text in keyword_lower:
                    matched_keyword = keyword
                    break
            
            if matched_keyword:
                self.get_logger().info(f'Target object keyword: "{matched_keyword}"')
                
                # Publish the recognized target object
                msg = String()
                msg.data = matched_keyword
                self.target_pub.publish(msg)
            else:
                self.get_logger().warn(
                    f'Recognized "{text}" not in vocabulary, ignoring'
                )
        else:
            # Partial result - log for debugging
            partial_text = result.get('partial', '')
            if partial_text:
                self.get_logger().debug(f'Partial: "{partial_text}"')


def main(args=None):
    """Main entry point for the voice command node."""
    rclpy.init(args=args)
    
    node = VoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


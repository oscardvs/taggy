#!/usr/bin/env python3
"""
Test Voice Command Node (Docker/No Audio)

Simulates audio input for testing voice_command_node in environments without
microphone access (e.g., Docker containers).

This script can:
1. Publish mock audio data (sine waves) - basic pipeline test
2. Publish audio from WAV file - realistic recognition test

Usage:
    # Terminal 1: Launch voice command node
    ros2 launch go2_mission voice_command.launch.py \
        model_path:=~/vosk-models/vosk-model-small-en-us-0.15

    # Terminal 2: Set LISTENING state
    ros2 topic pub /mission/state std_msgs/String "data: 'LISTENING'" -1

    # Terminal 3: Monitor output
    ros2 topic echo /mission/target_object

    # Terminal 4: Run test script
    # Option A: Mock audio (tests pipeline)
    python3 test_voice_command.py

    # Option B: Real audio file (tests recognition)
    python3 test_voice_command.py --audio-file /path/to/audio.wav
"""

import argparse
import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from unitree_go.msg import AudioData
    UNITREE_AVAILABLE = True
except ImportError:
    UNITREE_AVAILABLE = False
    print("Warning: unitree_go messages not available")

try:
    import wave
    WAV_AVAILABLE = True
except ImportError:
    WAV_AVAILABLE = False


class VoiceCommandTester(Node):
    """Publishes audio data to test voice recognition."""

    def __init__(self, audio_file=None):
        super().__init__('voice_command_tester')
        
        if not UNITREE_AVAILABLE:
            self.get_logger().error('unitree_go messages not available!')
            return

        # Publishers
        self.audio_pub = self.create_publisher(
            AudioData,
            '/audiohub/data',
            10
        )

        # Audio configuration
        self.sample_rate = 16000
        self.chunk_size = 1600  # 100ms of audio
        self.audio_counter = 0
        self.audio_data = None
        self.audio_index = 0

        # Load audio file if provided
        if audio_file and os.path.exists(audio_file):
            self._load_audio_file(audio_file)
        else:
            self.get_logger().info('Using mock audio (sine wave)')
            self.get_logger().warn(
                'Mock audio won\'t trigger recognition. '
                'Use --audio-file with a real WAV file for recognition testing.'
            )

        # Timer to publish audio chunks
        self.timer = self.create_timer(0.1, self.publish_audio)  # 10 Hz

        self.get_logger().info('Voice Command Tester started')
        self.get_logger().info('  Publishing to /audiohub/data')
        self.get_logger().info('  Note: Set /mission/state to LISTENING in another terminal')

    def _load_audio_file(self, audio_file):
        """Load audio from WAV file."""
        if not WAV_AVAILABLE:
            self.get_logger().error('wave module not available')
            return

        try:
            with wave.open(audio_file, 'rb') as wf:
                frames = wf.getnframes()
                sample_rate = wf.getframerate()
                channels = wf.getnchannels()
                sample_width = wf.getsampwidth()

                self.get_logger().info(f'Loaded audio file: {audio_file}')
                self.get_logger().info(f'  Sample rate: {sample_rate} Hz')
                self.get_logger().info(f'  Channels: {channels}')
                self.get_logger().info(f'  Duration: {frames/sample_rate:.2f} s')

                # Read all frames
                raw_audio = wf.readframes(frames)

                # Convert to numpy array
                if sample_width == 2:  # 16-bit
                    audio_array = np.frombuffer(raw_audio, dtype=np.int16)
                elif sample_width == 1:  # 8-bit
                    audio_array = np.frombuffer(raw_audio, dtype=np.int8).astype(np.int16) * 256
                else:
                    self.get_logger().error(f'Unsupported sample width: {sample_width}')
                    return

                # Convert to mono if stereo
                if channels == 2:
                    audio_array = audio_array.reshape(-1, 2).mean(axis=1).astype(np.int16)

                # Resample if needed (simple linear interpolation)
                if sample_rate != self.sample_rate:
                    ratio = self.sample_rate / sample_rate
                    num_samples = int(len(audio_array) * ratio)
                    indices = np.linspace(0, len(audio_array) - 1, num_samples)
                    audio_array = np.interp(indices, np.arange(len(audio_array)), audio_array).astype(np.int16)
                    self.get_logger().info(f'Resampled to {self.sample_rate} Hz')

                self.audio_data = audio_array
                self.get_logger().info(f'Audio loaded: {len(self.audio_data)} samples')

        except Exception as e:
            self.get_logger().error(f'Failed to load audio file: {e}')
            self.audio_data = None

    def publish_audio(self):
        """Publish audio data chunk."""
        if not UNITREE_AVAILABLE:
            return

        # Generate or use loaded audio
        if self.audio_data is not None:
            # Use loaded audio file
            if self.audio_index >= len(self.audio_data):
                self.get_logger().info('Audio playback complete')
                return
            
            end_idx = min(self.audio_index + self.chunk_size, len(self.audio_data))
            audio_samples = self.audio_data[self.audio_index:end_idx]
            self.audio_index = end_idx
        else:
            # Generate mock sine wave
            t = np.linspace(
                self.audio_counter * self.chunk_size / self.sample_rate,
                (self.audio_counter + 1) * self.chunk_size / self.sample_rate,
                self.chunk_size,
                False
            )
            audio_samples = (np.sin(2 * np.pi * 440 * t) * 0.1 * 32767).astype(np.int16)

        # Create AudioData message
        msg = AudioData()
        msg.time_frame = self.audio_counter
        msg.data = audio_samples.tobytes()

        self.audio_pub.publish(msg)
        self.audio_counter += 1

        # Log every 50 chunks (5 seconds)
        if self.audio_counter % 50 == 0:
            self.get_logger().info(f'Published {self.audio_counter} audio chunks')


def main():
    parser = argparse.ArgumentParser(
        description='Test voice command node with audio data (Docker-friendly)'
    )
    parser.add_argument(
        '--audio-file',
        type=str,
        default=None,
        help='Path to WAV audio file (16kHz mono recommended). If not provided, uses mock sine wave.'
    )
    parser.add_argument(
        '--duration',
        type=int,
        default=10,
        help='Test duration in seconds (default: 10, ignored if audio file provided)'
    )

    args = parser.parse_args()

    rclpy.init()
    node = VoiceCommandTester(audio_file=args.audio_file)

    try:
        if args.audio_file:
            # Run until audio finishes
            rclpy.spin(node)
        else:
            # Run for specified duration
            import time
            start_time = time.time()
            while time.time() - start_time < args.duration:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


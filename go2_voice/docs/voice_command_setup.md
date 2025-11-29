# Voice Command Setup Guide

Offline speech recognition for Go2 mission system using Vosk ASR.

## Dependencies

**System packages (Ubuntu/Debian):**
```bash
# Required for PyAudio (laptop testing only)
sudo apt-get install portaudio19-dev python3-pyaudio
```

**Python packages:**
```bash
pip install vosk numpy

# For laptop testing (local microphone)
# Note: Install portaudio19-dev first (see above)
pip install pyaudio
```

## Vosk Model Installation

```bash
# Create model directory
sudo mkdir -p /opt/vosk
cd /opt/vosk

# Download small English model (~50MB)
sudo wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
sudo unzip vosk-model-small-en-us-0.15.zip
sudo rm vosk-model-small-en-us-0.15.zip
```

Alternative model path (user home):
```bash
mkdir -p ~/vosk-models
cd ~/vosk-models
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
```

Then update `config/vosk_params.yaml`:
```yaml
model_path: '/home/YOUR_USER/vosk-models/vosk-model-small-en-us-0.15'
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select go2_mission
source install/setup.bash
```

## Launch

**Full mission stack (on robot):**
```bash
ros2 launch go2_mission mission.launch.py
```

**Voice command only (laptop testing with local mic):**
```bash
ros2 launch go2_mission voice_command.launch.py use_pyaudio:=true
```

**Voice command only (laptop, custom model path):**
```bash
ros2 launch go2_mission voice_command.launch.py \
    use_pyaudio:=true \
    model_path:=/home/YOUR_USER/vosk-models/vosk-model-small-en-us-0.15
```

## Testing

### On Robot or Laptop with Microphone

1. Publish mock mission state:
   ```bash
   ros2 topic pub /mission/state std_msgs/String "data: 'LISTENING'" -1
   ```

2. Monitor output:
   ```bash
   ros2 topic echo /mission/target_object
   ```

3. Speak a target object (e.g., "bottle", "backpack", "chair")

### In Docker Container (No Audio Access)

Since Docker containers typically don't have microphone access, use the test script:

**Terminal 1:** Launch voice command node (without PyAudio):
```bash
ros2 launch go2_mission voice_command.launch.py \
    model_path:=~/vosk-models/vosk-model-small-en-us-0.15
```

**Terminal 2:** Set LISTENING state:
```bash
ros2 topic pub /mission/state std_msgs/String "data: 'LISTENING'" -1
```

**Terminal 3:** Monitor recognized keywords:
```bash
ros2 topic echo /mission/target_object
```

**Terminal 4:** Publish audio data:
```bash
# Option A: Mock audio (tests pipeline, won't trigger recognition)
python3 ~/ros2_ws/src/go2_mission/scripts/test_voice_command.py

# Option B: Real audio file (tests actual recognition)
# First, record or download a WAV file saying a target word (e.g., "bottle")
# Format: 16kHz, mono, 16-bit WAV
python3 ~/ros2_ws/src/go2_mission/scripts/test_voice_command.py \
    --audio-file /path/to/bottle.wav
```

**Creating test audio files:**
- Use `ffmpeg` to convert any audio to 16kHz mono WAV:
  ```bash
  ffmpeg -i input.mp3 -ar 16000 -ac 1 -f wav output.wav
  ```
- Or use text-to-speech tools to generate test audio
- Or record on a system with microphone, then copy to Docker container

## Configuration

Edit `config/vosk_params.yaml` to:
- Change model path
- Modify vocabulary (must match COCO detection classes)
- Adjust audio topic for different microphone sources

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Vosk library not available" | `pip install vosk` |
| "Failed to load Vosk model" | Check `model_path` exists. Use absolute path or `~/` prefix |
| PyAudio build error: `portaudio.h: No such file` | `sudo apt-get install portaudio19-dev` then retry `pip install pyaudio` |
| PyAudio: "Invalid input device" in Docker | Expected - Docker containers don't have audio access. Use ROS topic mode instead |
| Model path not working with `$HOME` | Use `~` instead (e.g., `~/vosk-models/...`) or absolute path |
| No recognition output | Verify `/mission/state` is `LISTENING` |
| Poor accuracy | Reduce background noise, speak clearly |


# Deep Re-ID Tracker Implementation

## Overview

This implementation adds **deep learning-based re-identification** to the go2_tracker package, providing robust tracking of specific objects among multiple similar instances.

## What Was Implemented

### 1. **Deep Re-ID Tracker (`deep_reid_tracker.py`)**

A new tracker that uses neural networks for object re-identification:

- **OSNet (Omni-Scale Network)** - Pre-trained deep learning model for feature extraction
- **512D embeddings** - Compact discriminative feature vectors
- **Cosine similarity matching** - Compares embeddings to identify same object
- **Track ID memory** - Remembers target across brief occlusions (30 frames default)
- **Hybrid approach** - Trusts ByteTrack IDs + periodic Re-ID verification
- **Temporal persistence** - Handles objects leaving and re-entering frame

### 2. **Key Features**

#### Track Memory System
```python
# Remembers which ByteTrack ID corresponds to target
self.target_track_id = 42
self.track_last_seen[42] = frame_1234
self.track_embeddings[42] = embedding_vector
```

When target leaves frame:
- Keeps last known state for 30 frames (configurable)
- When it returns with same track ID → trusts ByteTrack
- When it returns with new track ID → re-verifies with Re-ID

#### Periodic Re-verification
```python
# Every 10 frames, verify we're still tracking correct object
if frame_number % 10 == 0:
    similarity = compare_embeddings(current, reference)
    if similarity < threshold:
        trigger_re_matching()
```

Prevents "drift" where tracker slowly switches to wrong object.

### 3. **Configuration Options**

New parameters in `tracker_config.yaml`:

```yaml
# Backend selection
reid_backend: "deep"  # or "classical"

# Deep Re-ID settings
reid_model: "osnet_x1_0"           # Architecture
reid_weights: "osnet_x1_0_imagenet" # Pre-trained weights
match_threshold: 0.6                # Similarity threshold
track_memory_frames: 30             # Memory duration
reid_verification_interval: 10      # Re-verify frequency
```

### 4. **Requirements**

New dependencies in `requirements.txt`:
- `torch>=2.0.0` - PyTorch deep learning framework
- `torchvision>=0.15.0` - Computer vision utilities
- `torchreid>=2.0.0` - Re-identification models library

## How It Works

### Algorithm Flow

```
1. DETECTION
   YOLO11 → Detects all objects of target class
   ByteTrack → Assigns persistent track IDs

2. TRACK ASSOCIATION
   If known target_track_id exists:
     ├─ Still visible? → Trust it (with periodic re-verification)
     └─ Disappeared? → Wait N frames (memory window)
   
   If no known target or lost:
     ├─ Extract embeddings for all detections
     ├─ Compare to reference embedding
     └─ Select best match above threshold

3. MEMORY UPDATE
   Store: track_id → embedding, last_seen_frame, match_score
```

### Example Scenario

**Scenario**: Track YOUR dog in a dog park with 5 dogs

```
Frame 1: 
  - Detect 5 dogs
  - Extract embeddings: [emb1, emb2, emb3, emb4, emb5]
  - Compare to reference (your dog's photo)
  - Best match: Dog #3 (similarity=0.87)
  - Lock: target_track_id = 3

Frames 2-100:
  - Track ID 3 still visible → trust ByteTrack
  - Frame 10, 20, 30... → re-verify (similarity still >0.7)
  - No Re-ID computation needed (efficient!)

Frame 101:
  - Your dog runs behind tree (occlusion)
  - Track ID 3 disappears
  - Keep waiting... (frames_since_seen = 1, 2, 3...)

Frame 115:
  - Your dog reappears
  - ByteTrack assigns Track ID 3 again (same object!)
  - Trust it immediately (no re-matching needed)

Frame 200:
  - Your dog leaves frame completely
  - Gone for 30 frames → memory timeout
  - target_track_id = None

Frame 250:
  - Your dog returns
  - ByteTrack assigns new Track ID 7 (lost memory)
  - Trigger Re-ID matching
  - Extract embedding for Track 7
  - Compare to reference: similarity = 0.91
  - Lock: target_track_id = 7 (re-identified!)
```

## Advantages Over Classical Re-ID

| Feature | Classical (SIFT+Histogram) | Deep Re-ID |
|---------|---------------------------|------------|
| **Viewpoint robustness** | ❌ Breaks on rotation | ✅ Handles any angle |
| **Lighting robustness** | ⚠️ Moderate | ✅ Excellent |
| **Occlusion handling** | ❌ Poor | ✅ Partial occlusion OK |
| **Similar objects** | ❌ Weak discrimination | ✅ Strong discrimination |
| **Memory across exits** | ❌ No memory | ✅ Track ID memory |
| **Speed** | ✅ Very fast (CPU) | ⚠️ Needs GPU for real-time |
| **Setup** | ✅ No dependencies | ⚠️ Requires torch/torchreid |

## Usage

### Installation

```bash
cd go2_tracker
pip install -r requirements.txt
```

For GPU support:
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### Configuration

Edit `config/tracker_config.yaml`:

```yaml
tracker:
  ros__parameters:
    threat_id: "dog"
    env_with_multiple_threats: true
    reid_backend: "deep"
    reference_image_path: "/path/to/reference.jpg"
    match_threshold: 0.6
    device: "0"  # Use GPU 0
```

### Run

```bash
ros2 launch go2_tracker tracker.launch.py
```

## Model Options

### Available Models

1. **osnet_x1_0** (Recommended)
   - 512D embeddings
   - Best balance of speed/accuracy
   - ~15-20 FPS on GPU

2. **osnet_x0_5** (Fast)
   - 512D embeddings
   - 2x faster than x1_0
   - Slightly lower accuracy

3. **resnet50** (Accurate)
   - 2048D embeddings
   - Highest accuracy
   - Slower inference

### Pre-trained Weights

1. **osnet_x1_0_imagenet** (General)
   - Works for any object class
   - Person, dog, cat, vehicle, etc.

2. **osnet_x1_0_market1501** (Person-specific)
   - Fine-tuned on person re-ID dataset
   - Best for tracking people

3. **osnet_x1_0_dukemtmc** (Person-specific)
   - Alternative person re-ID dataset
   - Good generalization

## Performance Tips

1. **Use GPU**: Deep Re-ID requires GPU for real-time performance (30 FPS)
2. **Adjust threshold**: Lower for challenging scenarios, higher for clean environments
3. **Tune memory**: Longer memory (60+ frames) for frequent occlusions
4. **Verification interval**: Lower (5) for crowded scenes, higher (20) for static scenes

## Limitations

1. **GPU requirement**: CPU-only will be slow (~5 FPS)
2. **Similar appearance**: Very similar objects may still confuse tracker
3. **Long absences**: If object gone >30 frames and ByteTrack loses ID, need re-matching
4. **Extreme changes**: Major appearance changes (e.g., dog gets wet) may break matching

## Future Improvements

- Multiple reference images (gallery-based matching)
- Online learning (adapt reference over time)
- Kalman filter for motion prediction
- Part-based matching for occlusion robustness
- Fine-tuning on custom datasets

---

**Author**: GitHub Copilot  
**Date**: November 29, 2025  
**Version**: 1.0

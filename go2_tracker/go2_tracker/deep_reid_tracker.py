#!/usr/bin/env python3
"""
Deep Learning Re-ID Tracker for Multi-Object Environments

This tracker uses deep neural networks for robust re-identification:
- YOLO11 for object detection (finds all objects of target class)
- Deep Re-ID embeddings for discriminative matching (identifies YOUR specific object)
- ByteTrack for temporal consistency (maintains track IDs across frames)
- Hybrid approach: trusts track IDs + periodic Re-ID verification

KEY IMPROVEMENTS OVER STANDARD RE-ID:
1. Deep learned features instead of hand-crafted (SIFT + histograms)
2. Track ID memory for persistence across brief occlusions
3. Robust to lighting, viewpoint, and pose variations
4. Can remember target even if it temporarily leaves frame

ALGORITHM OVERVIEW:
┌─────────────────────────────────────────────────────────────┐
│ 1. DETECTION PHASE                                          │
│    YOLO11 → Detects all objects of target class            │
│    ByteTrack → Assigns persistent track IDs                │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. RE-ID MATCHING PHASE                                     │
│    For each detection:                                      │
│      - Extract 512D embedding using OSNet/ResNet            │
│      - Compare to reference embedding (cosine similarity)   │
│      - Select best match above threshold                    │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. TRACK MEMORY PHASE                                       │
│    If target detected:                                      │
│      - Store track_id → embedding mapping                   │
│      - Trust this track_id for N frames                     │
│    If target lost:                                          │
│      - Check if known track_id reappears (ByteTrack)        │
│      - Re-verify with Re-ID if new track_id appears         │
└─────────────────────────────────────────────────────────────┘

USE WHEN:
- Multiple objects of same class in environment
- Need robust tracking across viewpoint/lighting changes  
- Object may temporarily leave and re-enter frame
- Higher computational resources available (GPU recommended)

USE STANDARD TRACKER WHEN:
- Only one object of target class expected
- Computational resources limited (CPU only)
- Simple, fast tracking is sufficient
"""

import cv2
import numpy as np
from typing import Optional, List, Dict, Tuple
from pathlib import Path
import time

import torch
import torch.nn.functional as F
from ultralytics import YOLO

try:
    import torchreid
    TORCHREID_AVAILABLE = True
except ImportError:
    TORCHREID_AVAILABLE = False
    print("WARNING: torchreid not available. Install with: pip install torchreid")

from go2_tracker.base_tracker import CameraIntrinsics, TrackedObject, pixel_to_3d
from go2_tracker.coco_classes import get_class_id


class DeepReIDTracker:
    """
    Deep learning-based Re-ID tracker with temporal memory.
    
    Uses pre-trained neural networks (OSNet, ResNet) to extract discriminative
    embeddings for robust object re-identification.
    
    ARCHITECTURE:
    - Detection: YOLO11 + ByteTrack
    - Re-ID: OSNet (Omni-Scale Network) or ResNet
    - Matching: Cosine similarity in embedding space
    - Memory: Track ID association with temporal persistence
    """
    
    def __init__(
        self,
        threat_id: str,
        reference_image_path: str = "",
        model: str = "yolo11n.pt",
        confidence_threshold: float = 0.5,
        reid_model: str = "osnet_x1_0",
        reid_weights: str = "osnet_x1_0_imagenet",
        match_threshold: float = 0.6,
        device: str = "",
        depth_filter_size: int = 5,
        track_memory_frames: int = 30,
        reid_verification_interval: int = 10,
    ):
        """
        Initialize Deep Re-ID tracker.
        
        Args:
            threat_id: Class name to track (e.g., "person", "dog")
            reference_image_path: Path to reference image of YOUR specific object
            model: YOLO11 model path (detection)
            confidence_threshold: Detection confidence threshold
            reid_model: Re-ID architecture ("osnet_x1_0", "osnet_x0_5", "resnet50")
            reid_weights: Pre-trained weights ("osnet_x1_0_imagenet", "osnet_x1_0_market1501")
            match_threshold: Minimum cosine similarity to accept match (0-1)
            device: Device ("", "cpu", "cuda", "0" for GPU 0)
            depth_filter_size: Median filter size for depth
            track_memory_frames: How long to remember disappeared tracks (0 = infinite)
            reid_verification_interval: Re-verify target every N frames
            
        Re-ID Model Options:
            - "osnet_x1_0": Best balance of speed/accuracy (recommended)
            - "osnet_x0_5": Faster, less accurate
            - "resnet50": More accurate, slower
            
        Re-ID Weights Options:
            - "osnet_x1_0_imagenet": General purpose (works for any object)
            - "osnet_x1_0_market1501": Fine-tuned for person re-ID
            - "osnet_x1_0_dukemtmc": Alternative person re-ID dataset
        """
        if not TORCHREID_AVAILABLE:
            raise ImportError(
                "torchreid is required for DeepReIDTracker. "
                "Install with: pip install torchreid"
            )
        
        self.threat_id = threat_id
        self.target_class = get_class_id(threat_id)
        self.conf_threshold = confidence_threshold
        self.match_threshold = match_threshold
        self.depth_filter_size = depth_filter_size
        self.track_memory_frames = track_memory_frames
        self.reid_verify_interval = reid_verification_interval
        
        if self.target_class < 0:
            raise ValueError(f"Unknown class name: {threat_id}")
        
        # Device setup
        if not device:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = torch.device(device)
        
        print(f"Using device: {self.device}")
        
        # Load YOLO11 for detection
        self.yolo = YOLO(model)
        if str(self.device) != 'cpu':
            self.yolo.to(self.device)
        
        # Warmup YOLO
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.yolo.track(dummy, persist=True, verbose=False)
        
        # Load Re-ID model
        print(f"Loading Re-ID model: {reid_model} with {reid_weights}")
        self.reid_model = torchreid.models.build_model(
            name=reid_model,
            num_classes=1000,  # Doesn't matter for feature extraction
            pretrained=True,
            loss='softmax'
        )
        
        # Load pretrained weights if specified
        if reid_weights and reid_weights != "imagenet":
            try:
                # Try to load from torchreid model zoo
                torchreid.utils.load_pretrained_weights(self.reid_model, reid_weights)
            except:
                print(f"Warning: Could not load weights '{reid_weights}', using ImageNet pretrained")
        
        self.reid_model = self.reid_model.to(self.device)
        self.reid_model.eval()  # Inference mode
        
        # Re-ID preprocessing (standard ImageNet normalization)
        self.reid_transform_mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1).to(self.device)
        self.reid_transform_std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1).to(self.device)
        
        # Reference embedding
        self.reference_embedding: Optional[torch.Tensor] = None
        self.reference_image: Optional[np.ndarray] = None
        
        if reference_image_path and Path(reference_image_path).exists():
            self.load_reference(reference_image_path)
        
        # Track state management
        self.target_track_id: Optional[int] = None  # The track ID of our target
        self.track_embeddings: Dict[int, torch.Tensor] = {}  # track_id → embedding
        self.track_last_seen: Dict[int, int] = {}  # track_id → frame_number
        self.track_match_scores: Dict[int, float] = {}  # track_id → best match score
        
        # Current frame state
        self.current_target: Optional[TrackedObject] = None
        self.all_detections: List[TrackedObject] = []
        self.frame_number = 0
        
        # Performance tracking
        self.inference_times: List[float] = []
        self.reid_times: List[float] = []
        
        print(f"DeepReIDTracker initialized for '{threat_id}'")
        print(f"  Match threshold: {match_threshold}")
        print(f"  Track memory: {track_memory_frames} frames")
        print(f"  Re-ID verification interval: {reid_verify_interval} frames")
    
    def load_reference(self, image_path: str) -> bool:
        """
        Load reference image and extract embedding.
        
        Args:
            image_path: Path to reference image
            
        Returns:
            True if successful
        """
        image = cv2.imread(image_path)
        if image is None:
            print(f"Error: Could not load reference image from {image_path}")
            return False
        
        return self.set_reference(image)
    
    def set_reference(self, image: np.ndarray) -> bool:
        """
        Set reference image directly and extract embedding.
        
        Args:
            image: BGR reference image
            
        Returns:
            True if successful
        """
        if image is None or image.size == 0:
            return False
        
        self.reference_image = image.copy()
        
        # Extract deep embedding
        self.reference_embedding = self._extract_embedding(image)
        
        if self.reference_embedding is not None:
            print(f"Reference embedding extracted: shape {self.reference_embedding.shape}")
            return True
        else:
            print("Error: Failed to extract reference embedding")
            return False
    
    def has_reference(self) -> bool:
        """Check if reference embedding is set."""
        return self.reference_embedding is not None
    
    def _extract_embedding(self, image: np.ndarray) -> Optional[torch.Tensor]:
        """
        Extract Re-ID embedding from image using deep neural network.
        
        This is the core Re-ID operation that converts an image into a 
        discriminative feature vector.
        
        Args:
            image: BGR image (full image or cropped ROI)
            
        Returns:
            Normalized embedding tensor (512D or 2048D) or None if failed
        """
        if image is None or image.size == 0:
            return None
        
        try:
            # Preprocessing: BGR → RGB, resize to 256x128 (Re-ID standard)
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (128, 256))  # Width × Height
            
            # Convert to tensor: HWC → CHW, normalize to [0, 1]
            tensor = torch.from_numpy(resized).permute(2, 0, 1).float() / 255.0
            tensor = tensor.unsqueeze(0).to(self.device)  # Add batch dimension
            
            # ImageNet normalization
            tensor = (tensor - self.reid_transform_mean) / self.reid_transform_std
            
            # Extract features (no gradient computation)
            with torch.no_grad():
                features = self.reid_model(tensor)
            
            # L2 normalization (for cosine similarity)
            embedding = F.normalize(features, p=2, dim=1)
            
            return embedding.squeeze(0)  # Remove batch dimension
            
        except Exception as e:
            print(f"Error extracting embedding: {e}")
            return None
    
    def _compute_similarity(self, embedding1: torch.Tensor, embedding2: torch.Tensor) -> float:
        """
        Compute cosine similarity between two embeddings.
        
        Cosine similarity measures the angle between two vectors:
        - 1.0 = identical direction (same object)
        - 0.0 = orthogonal (unrelated)
        - -1.0 = opposite direction (very different)
        
        Args:
            embedding1, embedding2: Normalized embedding tensors
            
        Returns:
            Similarity score in [0, 1]
        """
        if embedding1 is None or embedding2 is None:
            return 0.0
        
        # Cosine similarity (already normalized, so just dot product)
        similarity = torch.dot(embedding1, embedding2).item()
        
        # Clamp to [0, 1] range
        return max(0.0, min(1.0, similarity))
    
    def process_frame(
        self,
        rgb_image: np.ndarray,
        depth_image: Optional[np.ndarray],
        intrinsics: Optional[CameraIntrinsics]
    ) -> Optional[TrackedObject]:
        """
        Process a frame with hybrid Re-ID + track ID approach.
        
        ALGORITHM:
        1. Run YOLO11 + ByteTrack to get detections with track IDs
        2. If we have a known target_track_id:
           a. Check if it still exists in current frame
           b. If yes, trust it (with periodic re-verification)
           c. If no, check if it recently disappeared (memory)
        3. If no known target or lost for too long:
           a. Extract embeddings for all detections
           b. Match against reference using cosine similarity
           c. Select best match above threshold
        4. Update track memory and associations
        
        Args:
            rgb_image: BGR image
            depth_image: Depth image (mm) or None
            intrinsics: Camera intrinsics or None
            
        Returns:
            Tracked target object or None
        """
        if not self.has_reference():
            return None
        
        start_time = time.time()
        self.frame_number += 1
        
        # ========================================
        # STEP 1: Detection with ByteTrack
        # ========================================
        results = self.yolo.track(
            rgb_image,
            persist=True,
            conf=self.conf_threshold,
            classes=[self.target_class],
            tracker='bytetrack.yaml',
            verbose=False
        )
        
        # Parse detections
        self.all_detections = []
        current_track_ids = set()
        
        if results and len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            if boxes.id is not None:
                track_ids = boxes.id.cpu().numpy().astype(int)
                class_ids = boxes.cls.cpu().numpy().astype(int)
                confidences = boxes.conf.cpu().numpy()
                xyxy = boxes.xyxy.cpu().numpy().astype(int)
                
                for i, track_id in enumerate(track_ids):
                    current_track_ids.add(track_id)
                    
                    x1, y1, x2, y2 = xyxy[i]
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # Get 3D position
                    position_3d = None
                    if depth_image is not None and intrinsics is not None:
                        position_3d = pixel_to_3d(
                            cx, cy, depth_image, intrinsics, self.depth_filter_size
                        )
                    
                    detection = TrackedObject(
                        track_id=track_id,
                        class_id=class_ids[i],
                        class_name=self.threat_id,
                        bbox=(x1, y1, x2, y2),
                        center_pixel=(cx, cy),
                        confidence=confidences[i],
                        position_3d=position_3d,
                        match_score=0.0,  # Will be computed if needed
                    )
                    
                    self.all_detections.append(detection)
        
        # ========================================
        # STEP 2: Track ID Association Logic
        # ========================================
        target_found = False
        need_reid_matching = False
        
        # Check if our known target is still visible
        if self.target_track_id is not None:
            if self.target_track_id in current_track_ids:
                # Target track ID still exists!
                target_detection = next(
                    d for d in self.all_detections if d.track_id == self.target_track_id
                )
                
                # Periodic re-verification (prevent drift/mis-association)
                if self.frame_number % self.reid_verify_interval == 0:
                    reid_start = time.time()
                    roi = rgb_image[target_detection.bbox[1]:target_detection.bbox[3],
                                   target_detection.bbox[0]:target_detection.bbox[2]]
                    embedding = self._extract_embedding(roi)
                    
                    if embedding is not None:
                        similarity = self._compute_similarity(self.reference_embedding, embedding)
                        target_detection.match_score = similarity
                        self.reid_times.append(time.time() - reid_start)
                        
                        if similarity < self.match_threshold * 0.7:  # Lower threshold for verification
                            print(f"WARNING: Track {self.target_track_id} failed re-verification (score={similarity:.2f})")
                            need_reid_matching = True
                        else:
                            self.current_target = target_detection
                            self.track_last_seen[self.target_track_id] = self.frame_number
                            target_found = True
                    else:
                        need_reid_matching = True
                else:
                    # Trust the track ID
                    self.current_target = target_detection
                    self.track_last_seen[self.target_track_id] = self.frame_number
                    target_found = True
            else:
                # Target track ID disappeared - check if recently
                frames_since_seen = self.frame_number - self.track_last_seen.get(self.target_track_id, 0)
                
                # Check memory window (0 = infinite memory)
                memory_expired = (self.track_memory_frames > 0 and 
                                frames_since_seen >= self.track_memory_frames)
                
                if not memory_expired:
                    # Keep waiting (brief occlusion or infinite memory)
                    # Don't update current_target (keep last known)
                    target_found = False
                else:
                    # Lost for too long, need to re-match
                    print(f"Track {self.target_track_id} lost for {frames_since_seen} frames, re-matching...")
                    self.target_track_id = None
                    need_reid_matching = True
        else:
            # No known target, need Re-ID matching
            need_reid_matching = True
        
        # ========================================
        # STEP 3: Deep Re-ID Matching
        # ========================================
        if need_reid_matching and self.all_detections:
            reid_start = time.time()
            
            best_match = None
            best_score = 0.0
            
            for detection in self.all_detections:
                # Extract ROI
                x1, y1, x2, y2 = detection.bbox
                roi = rgb_image[y1:y2, x1:x2]
                
                if roi.size == 0:
                    continue
                
                # Extract embedding
                embedding = self._extract_embedding(roi)
                
                if embedding is not None:
                    # Compute similarity to reference
                    similarity = self._compute_similarity(self.reference_embedding, embedding)
                    detection.match_score = similarity
                    
                    # Store embedding for this track
                    self.track_embeddings[detection.track_id] = embedding
                    
                    if similarity > best_score:
                        best_score = similarity
                        best_match = detection
            
            self.reid_times.append(time.time() - reid_start)
            
            # Accept match if above threshold
            if best_match and best_score >= self.match_threshold:
                self.current_target = best_match
                self.target_track_id = best_match.track_id
                self.track_last_seen[self.target_track_id] = self.frame_number
                self.track_match_scores[self.target_track_id] = best_score
                target_found = True
                print(f"New target locked: Track ID {self.target_track_id}, similarity={best_score:.3f}")
            else:
                self.current_target = None
                target_found = False
        
        # Track inference time
        self.inference_times.append(time.time() - start_time)
        if len(self.inference_times) > 100:
            self.inference_times.pop(0)
        if len(self.reid_times) > 100:
            self.reid_times.pop(0)
        
        return self.current_target if target_found else None
    
    def get_debug_image(self, rgb_image: np.ndarray) -> np.ndarray:
        """
        Generate debug visualization with Re-ID scores and track IDs.
        
        Color coding:
        - Magenta box: Current target being tracked
        - Green: High Re-ID similarity (> threshold)
        - Yellow: Medium similarity
        - Red: Low similarity
        """
        debug_img = rgb_image.copy()
        
        # Draw all detections
        for det in self.all_detections:
            x1, y1, x2, y2 = det.bbox
            
            # Color based on match score
            is_target = (self.target_track_id is not None and 
                        det.track_id == self.target_track_id)
            
            if is_target:
                color = (255, 0, 255)  # Magenta - target
                thickness = 4
            elif det.match_score >= self.match_threshold:
                color = (0, 255, 0)  # Green - good match
                thickness = 2
            elif det.match_score >= self.match_threshold * 0.7:
                color = (0, 255, 255)  # Yellow - medium
                thickness = 2
            else:
                color = (0, 0, 255)  # Red - poor match
                thickness = 2
            
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, thickness)
            
            # Label with track ID and score
            label = f"ID:{det.track_id}"
            if det.match_score > 0:
                label += f" {det.match_score:.2f}"
            
            cv2.putText(debug_img, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 3D position
            if det.position_3d is not None:
                pos = det.position_3d
                pos_text = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})m"
                cv2.putText(debug_img, pos_text, (x1, y2 + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Highlight current target with extra box
        if self.current_target:
            x1, y1, x2, y2 = self.current_target.bbox
            cv2.rectangle(debug_img, (x1-5, y1-5), (x2+5, y2+5), (255, 0, 255), 3)
            cv2.putText(debug_img, "TARGET", (x1, y1 - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        
        # Show reference in corner
        if self.reference_image is not None:
            ref_h, ref_w = 100, 100
            ref_small = cv2.resize(self.reference_image, (ref_w, ref_h))
            debug_img[10:10+ref_h, 10:10+ref_w] = ref_small
            cv2.rectangle(debug_img, (10, 10), (10+ref_w, 10+ref_h), (255, 255, 255), 2)
            cv2.putText(debug_img, "REFERENCE", (15, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Info overlay
        fps = 1.0 / np.mean(self.inference_times) if self.inference_times else 0
        reid_time = np.mean(self.reid_times) * 1000 if self.reid_times else 0
        
        info_y = debug_img.shape[0] - 70
        cv2.putText(debug_img, f"Deep Re-ID | {self.threat_id}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_img, f"FPS: {fps:.1f} | Re-ID: {reid_time:.1f}ms", 
                   (10, info_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_img, f"Tracks: {len(self.all_detections)} | Target ID: {self.target_track_id or 'None'}", 
                   (10, info_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return debug_img
    
    def get_status(self) -> str:
        """Get current tracker status string."""
        if not self.has_reference():
            return "No reference image set - run capture_reference first"
        
        n = len(self.all_detections)
        
        if self.current_target:
            pos = self.current_target.position_3d
            score = self.current_target.match_score
            tid = self.current_target.track_id
            
            if pos is not None:
                return (f"Tracking YOUR {self.threat_id} | ID={tid} | "
                       f"Match: {score:.2f} | 3D: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})m")
            else:
                return f"Tracking YOUR {self.threat_id} | ID={tid} | Match: {score:.2f} | No depth"
        else:
            if n > 0:
                best_score = max((d.match_score for d in self.all_detections), default=0)
                return f"Searching for YOUR {self.threat_id}... ({n} detected, best: {best_score:.2f})"
            else:
                return f"No {self.threat_id} detected"

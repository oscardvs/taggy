#!/usr/bin/env python3
"""
Re-ID Tracker for Multi-Threat Environments

Tracks a SPECIFIC object instance among multiple similar objects using:
1. YOLO11 for class detection (find all dogs)
2. Re-ID features to identify the specific target (find YOUR dog)

Use this when MULTIPLE objects of the target class may be present.
"""

import cv2
import numpy as np
from typing import Optional, List, Tuple
from enum import Enum
import time

from ultralytics import YOLO

from go2_tracker.base_tracker import CameraIntrinsics, TrackedObject, pixel_to_3d
from go2_tracker.coco_classes import get_class_id


class ReIDMethod(Enum):
    HISTOGRAM = "histogram"
    SIFT = "sift"
    COMBINED = "combined"


class ReIDTracker:
    """
    Re-identification based tracker for specific object instances.
    
    Identifies and tracks YOUR specific object among multiple similar ones.
    """
    
    def __init__(
        self,
        threat_id: str,
        reference_image_path: str,
        model: str = "yolo11n.pt",
        confidence_threshold: float = 0.5,
        reid_method: str = "combined",
        match_threshold: float = 0.5,
        device: str = "",
        depth_filter_size: int = 5,
    ):
        """
        Initialize the Re-ID tracker.
        
        Args:
            threat_id: Class name to track (e.g., "dog")
            reference_image_path: Path to reference image of YOUR specific object
            model: YOLO11 model path
            confidence_threshold: Detection confidence threshold
            reid_method: Matching method (histogram, sift, combined)
            match_threshold: Minimum match score to accept
            device: Device to run on
            depth_filter_size: Median filter size for depth
        """
        self.threat_id = threat_id
        self.target_class = get_class_id(threat_id)
        self.conf_threshold = confidence_threshold
        self.reid_method = ReIDMethod(reid_method)
        self.match_threshold = match_threshold
        self.depth_filter_size = depth_filter_size
        
        if self.target_class < 0:
            raise ValueError(f"Unknown class name: {threat_id}")
        
        # Load YOLO11
        self.model = YOLO(model)
        if device:
            self.model.to(device)
        
        # Warmup
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.model(dummy, verbose=False)
        
        # Initialize Re-ID features
        self.sift = cv2.SIFT_create()
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_L2)
        
        # Reference features
        self.reference_image: Optional[np.ndarray] = None
        self.reference_histogram: Optional[np.ndarray] = None
        self.reference_sift_desc: Optional[np.ndarray] = None
        
        # Load reference
        if reference_image_path:
            self.load_reference(reference_image_path)
        
        # State
        self.current_target: Optional[TrackedObject] = None
        self.all_detections: List[TrackedObject] = []
        self.inference_times: List[float] = []
        self.frames_since_match = 0
        self.track_persistence = 10
    
    def load_reference(self, image_path: str) -> bool:
        """
        Load reference image for the specific object to track.
        
        Args:
            image_path: Path to reference image
            
        Returns:
            True if successful
        """
        image = cv2.imread(image_path)
        if image is None:
            return False
        
        return self.set_reference(image)
    
    def set_reference(self, image: np.ndarray) -> bool:
        """
        Set reference image directly.
        
        Args:
            image: BGR reference image
            
        Returns:
            True if successful
        """
        if image is None or image.size == 0:
            return False
        
        self.reference_image = image.copy()
        
        # Compute color histogram (HSV for illumination robustness)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.reference_histogram = cv2.calcHist(
            [hsv], [0, 1, 2], None, [32, 32, 32], [0, 180, 0, 256, 0, 256]
        )
        cv2.normalize(self.reference_histogram, self.reference_histogram, 0, 1, cv2.NORM_MINMAX)
        
        # Compute SIFT features
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, self.reference_sift_desc = self.sift.detectAndCompute(gray, None)
        
        return True
    
    def has_reference(self) -> bool:
        """Check if reference image is set."""
        return self.reference_histogram is not None
    
    def process_frame(
        self,
        rgb_image: np.ndarray,
        depth_image: Optional[np.ndarray],
        intrinsics: Optional[CameraIntrinsics]
    ) -> Optional[TrackedObject]:
        """
        Process a frame and return the matched target object.
        
        Args:
            rgb_image: BGR image
            depth_image: Depth image (mm) or None
            intrinsics: Camera intrinsics or None
            
        Returns:
            Matched target object or None
        """
        if not self.has_reference():
            return None
        
        start_time = time.time()
        
        # Detect all objects of target class
        results = self.model(
            rgb_image,
            conf=self.conf_threshold,
            classes=[self.target_class],
            verbose=False
        )
        
        # Process detections
        self.all_detections = []
        
        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            
            for i in range(len(boxes)):
                xyxy = boxes.xyxy[i].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Extract ROI for matching
                roi = rgb_image[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                
                # Compute match score
                match_score = self._compute_match_score(roi)
                
                # Get 3D position
                position_3d = None
                if depth_image is not None and intrinsics is not None:
                    position_3d = pixel_to_3d(
                        cx, cy, depth_image, intrinsics, self.depth_filter_size
                    )
                
                det = TrackedObject(
                    track_id=i,
                    class_id=self.target_class,
                    class_name=self.threat_id,
                    bbox=(x1, y1, x2, y2),
                    center_pixel=(cx, cy),
                    confidence=float(boxes.conf[i].cpu().numpy()),
                    position_3d=position_3d,
                    match_score=match_score,
                )
                
                self.all_detections.append(det)
        
        # Find best match
        best_match = None
        best_score = 0
        
        for det in self.all_detections:
            if det.match_score > best_score:
                best_score = det.match_score
                best_match = det
        
        # Update target
        if best_match and best_match.match_score >= self.match_threshold:
            self.current_target = best_match
            self.frames_since_match = 0
        else:
            self.frames_since_match += 1
            if self.frames_since_match > self.track_persistence:
                self.current_target = None
        
        # Track inference time
        self.inference_times.append(time.time() - start_time)
        if len(self.inference_times) > 100:
            self.inference_times.pop(0)
        
        return self.current_target
    
    def _compute_match_score(self, roi: np.ndarray) -> float:
        """Compute similarity between ROI and reference."""
        scores = []
        weights = []
        
        # Histogram matching
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1, 2], None, [32, 32, 32], [0, 180, 0, 256, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        
        hist_score = cv2.compareHist(self.reference_histogram, hist, cv2.HISTCMP_CORREL)
        hist_score = max(0, hist_score)
        scores.append(hist_score)
        weights.append(0.4)
        
        # SIFT matching
        if self.reference_sift_desc is not None and self.reid_method in [ReIDMethod.SIFT, ReIDMethod.COMBINED]:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, desc = self.sift.detectAndCompute(gray, None)
            
            if desc is not None and len(desc) >= 2:
                try:
                    matches = self.bf_matcher.knnMatch(self.reference_sift_desc, desc, k=2)
                    good = []
                    for match in matches:
                        if len(match) == 2:
                            m, n = match
                            if m.distance < 0.75 * n.distance:
                                good.append(m)
                    
                    sift_score = min(len(good) / 15.0, 1.0)
                    scores.append(sift_score)
                    weights.append(0.6)
                except:
                    pass
        
        if not scores:
            return 0.0
        
        # Weighted average
        total_weight = sum(weights)
        return sum(s * w for s, w in zip(scores, weights)) / total_weight
    
    def get_debug_image(self, rgb_image: np.ndarray) -> np.ndarray:
        """Generate debug visualization."""
        debug_img = rgb_image.copy()
        
        # Draw all detections
        for det in self.all_detections:
            x1, y1, x2, y2 = det.bbox
            
            # Color based on match score
            if det.match_score >= self.match_threshold:
                color = (0, 255, 0)  # Green - matched
                thickness = 3
            elif det.match_score >= self.match_threshold * 0.7:
                color = (0, 255, 255)  # Yellow - close
                thickness = 2
            else:
                color = (0, 0, 255)  # Red - not matching
                thickness = 2
            
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, thickness)
            
            # Match score
            cv2.putText(debug_img, f"{det.match_score:.2f}", (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Highlight current target
        if self.current_target:
            x1, y1, x2, y2 = self.current_target.bbox
            cv2.rectangle(debug_img, (x1-4, y1-4), (x2+4, y2+4), (255, 0, 255), 4)
            
            if self.current_target.position_3d is not None:
                pos = self.current_target.position_3d
                cv2.putText(debug_img, f"TARGET: {pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}m",
                           (x1, y2 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # Show reference in corner
        if self.reference_image is not None:
            ref_h, ref_w = 80, 80
            ref_small = cv2.resize(self.reference_image, (ref_w, ref_h))
            debug_img[10:10+ref_h, 10:10+ref_w] = ref_small
            cv2.rectangle(debug_img, (10, 10), (10+ref_w, 10+ref_h), (255, 255, 255), 2)
            cv2.putText(debug_img, "REF", (15, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Info overlay
        fps = 1.0 / np.mean(self.inference_times) if self.inference_times else 0
        cv2.putText(debug_img, f"Re-ID | {self.threat_id} | FPS: {fps:.1f}", 
                   (10, debug_img.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_img, f"Detections: {len(self.all_detections)} | Threshold: {self.match_threshold}", 
                   (10, debug_img.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return debug_img
    
    def get_status(self) -> str:
        """Get current tracker status string."""
        n = len(self.all_detections)
        
        if not self.has_reference():
            return "No reference image set - run capture_reference first"
        
        if self.current_target:
            pos = self.current_target.position_3d
            if pos is not None:
                return (f"Tracking YOUR {self.threat_id} | Match: {self.current_target.match_score:.2f} | "
                       f"3D: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})m")
            else:
                return f"Tracking YOUR {self.threat_id} | Match: {self.current_target.match_score:.2f} | No depth"
        else:
            best_score = max((d.match_score for d in self.all_detections), default=0)
            return f"Searching for YOUR {self.threat_id}... ({n} detected, best match: {best_score:.2f})"
#!/usr/bin/env python3
"""
Standard YOLO11 Tracker

Tracks objects of a specified class using YOLO11 + ByteTrack.
Use this when only ONE object of the target class is expected in the environment.

For environments with multiple objects of the same class, use ReIDTracker instead.
"""

import cv2
import numpy as np
from typing import Optional, Dict, List
import time

from ultralytics import YOLO

from go2_tracker.base_tracker import CameraIntrinsics, TrackedObject, pixel_to_3d
from go2_tracker.coco_classes import get_class_id, get_class_name


class StandardTracker:
    """
    Standard YOLO11 object tracker.
    
    Tracks the closest object of the specified class.
    """
    
    def __init__(
        self,
        threat_id: str,
        model: str = "yolo11n.pt",
        confidence_threshold: float = 0.5,
        tracker_type: str = "bytetrack.yaml",
        device: str = "",
        depth_filter_size: int = 5,
    ):
        """
        Initialize the standard tracker.
        
        Args:
            threat_id: Class name to track (e.g., "dog", "person")
            model: YOLO11 model path
            confidence_threshold: Detection confidence threshold
            tracker_type: Tracker config (bytetrack.yaml or botsort.yaml)
            device: Device to run on ("", "cpu", or "0" for GPU)
            depth_filter_size: Median filter size for depth
        """
        self.threat_id = threat_id
        self.target_class = get_class_id(threat_id)
        self.conf_threshold = confidence_threshold
        self.tracker_type = tracker_type
        self.depth_filter_size = depth_filter_size
        
        if self.target_class < 0:
            raise ValueError(f"Unknown class name: {threat_id}")
        
        # Load YOLO11 model
        self.model = YOLO(model)
        if device:
            self.model.to(device)
        
        # Warmup
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.model.track(dummy, persist=True, verbose=False)
        
        # State
        self.tracked_objects: Dict[int, TrackedObject] = {}
        self.primary_target: Optional[TrackedObject] = None
        self.inference_times: List[float] = []
    
    def process_frame(
        self,
        rgb_image: np.ndarray,
        depth_image: Optional[np.ndarray],
        intrinsics: Optional[CameraIntrinsics]
    ) -> Optional[TrackedObject]:
        """
        Process a frame and return the primary tracked object.
        
        Args:
            rgb_image: BGR image
            depth_image: Depth image (mm) or None
            intrinsics: Camera intrinsics or None
            
        Returns:
            Primary tracked object or None
        """
        start_time = time.time()
        
        # Run YOLO11 tracking
        results = self.model.track(
            rgb_image,
            persist=True,
            conf=self.conf_threshold,
            classes=[self.target_class],
            tracker=self.tracker_type,
            verbose=False
        )
        
        # Process results
        current_tracks = {}
        
        if results and len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            if boxes.id is not None:
                track_ids = boxes.id.cpu().numpy().astype(int)
                class_ids = boxes.cls.cpu().numpy().astype(int)
                confidences = boxes.conf.cpu().numpy()
                xyxy = boxes.xyxy.cpu().numpy().astype(int)
                
                for i, track_id in enumerate(track_ids):
                    x1, y1, x2, y2 = xyxy[i]
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # Get 3D position
                    position_3d = None
                    if depth_image is not None and intrinsics is not None:
                        position_3d = pixel_to_3d(
                            cx, cy, depth_image, intrinsics, self.depth_filter_size
                        )
                    
                    tracked_obj = TrackedObject(
                        track_id=track_id,
                        class_id=class_ids[i],
                        class_name=self.threat_id,
                        bbox=(x1, y1, x2, y2),
                        center_pixel=(cx, cy),
                        confidence=confidences[i],
                        position_3d=position_3d,
                    )
                    
                    current_tracks[track_id] = tracked_obj
        
        self.tracked_objects = current_tracks
        
        # Select primary target (closest with valid depth)
        self.primary_target = self._select_primary_target()
        
        # Track inference time
        self.inference_times.append(time.time() - start_time)
        if len(self.inference_times) > 100:
            self.inference_times.pop(0)
        
        return self.primary_target
    
    def _select_primary_target(self) -> Optional[TrackedObject]:
        """Select the closest object with valid depth."""
        if not self.tracked_objects:
            return None
        
        closest_dist = float('inf')
        closest_obj = None
        
        for obj in self.tracked_objects.values():
            if obj.position_3d is not None:
                dist = np.linalg.norm(obj.position_3d)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_obj = obj
        
        # If no object with depth, return first one
        if closest_obj is None and self.tracked_objects:
            closest_obj = list(self.tracked_objects.values())[0]
        
        return closest_obj
    
    def get_debug_image(self, rgb_image: np.ndarray) -> np.ndarray:
        """Generate debug visualization."""
        debug_img = rgb_image.copy()
        
        for obj in self.tracked_objects.values():
            x1, y1, x2, y2 = obj.bbox
            
            # Color based on whether it's the primary target
            if self.primary_target and obj.track_id == self.primary_target.track_id:
                color = (0, 255, 0)  # Green
                thickness = 3
            else:
                color = (0, 165, 255)  # Orange
                thickness = 2
            
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, thickness)
            
            # Label
            label = f"ID:{obj.track_id} {obj.confidence:.2f}"
            cv2.putText(debug_img, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 3D position
            if obj.position_3d is not None:
                pos = obj.position_3d
                pos_text = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})m"
                cv2.putText(debug_img, pos_text, (x1, y2 + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Info overlay
        fps = 1.0 / np.mean(self.inference_times) if self.inference_times else 0
        cv2.putText(debug_img, f"YOLO11 | {self.threat_id} | FPS: {fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(debug_img, f"Tracks: {len(self.tracked_objects)}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return debug_img
    
    def get_status(self) -> str:
        """Get current tracker status string."""
        n = len(self.tracked_objects)
        
        if self.primary_target:
            pos = self.primary_target.position_3d
            if pos is not None:
                return (f"Tracking {self.threat_id} | ID={self.primary_target.track_id} | "
                       f"3D: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})m")
            else:
                return f"Tracking {self.threat_id} | ID={self.primary_target.track_id} | No depth"
        else:
            return f"Searching for {self.threat_id}... ({n} detections)"
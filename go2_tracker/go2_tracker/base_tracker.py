#!/usr/bin/env python3
"""
Base Tracker Module

Contains shared functionality for all tracker implementations.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import time


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class TrackedObject:
    """Represents a tracked object."""
    track_id: int
    class_id: int
    class_name: str
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center_pixel: Tuple[int, int]
    confidence: float
    position_3d: Optional[np.ndarray] = None
    match_score: float = 1.0
    last_seen: float = field(default_factory=time.time)
    
    @property
    def center(self) -> Tuple[int, int]:
        return self.center_pixel


def pixel_to_3d(
    u: int, 
    v: int, 
    depth_image: np.ndarray,
    intrinsics: CameraIntrinsics,
    filter_size: int = 5
) -> Optional[np.ndarray]:
    """
    Convert pixel coordinates to 3D point using depth.
    
    Back-projection equations:
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth
    
    Args:
        u, v: Pixel coordinates
        depth_image: Depth image (mm)
        intrinsics: Camera intrinsic parameters
        filter_size: Median filter window size
        
    Returns:
        3D point [X, Y, Z] in meters or None if invalid
    """
    if depth_image is None or intrinsics is None:
        return None
    
    h, w = depth_image.shape
    u = max(0, min(u, w - 1))
    v = max(0, min(v, h - 1))
    
    # Median filter for robust depth
    half = filter_size // 2
    u_min, u_max = max(0, u - half), min(w, u + half + 1)
    v_min, v_max = max(0, v - half), min(h, v + half + 1)
    
    depth_region = depth_image[v_min:v_max, u_min:u_max]
    valid_depths = depth_region[depth_region > 0]
    
    if len(valid_depths) == 0:
        return None
    
    depth_mm = np.median(valid_depths)
    Z = depth_mm / 1000.0  # mm to meters
    
    # Filter unreasonable depths
    if Z < 0.1 or Z > 10.0:
        return None
    
    # Back-projection
    X = (u - intrinsics.cx) * Z / intrinsics.fx
    Y = (v - intrinsics.cy) * Z / intrinsics.fy
    
    return np.array([X, Y, Z])
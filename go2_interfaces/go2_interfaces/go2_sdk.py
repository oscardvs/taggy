#!/usr/bin/env python3
"""
Unitree Go2 SDK Wrapper
Provides high-level interface to Go2 robot via UDP communication

This is a simplified interface for the hackathon. The actual Unitree SDK
uses UDP protocol to communicate with the robot's onboard computer.
"""

import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable
import math


@dataclass
class RobotState:
    """Robot state data from Go2"""
    # Position (from leg odometry)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    # Orientation (quaternion)
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    
    # Velocity
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    
    # Angular velocity
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    
    # IMU data
    imu_acc_x: float = 0.0
    imu_acc_y: float = 0.0
    imu_acc_z: float = 0.0
    imu_gyro_x: float = 0.0
    imu_gyro_y: float = 0.0
    imu_gyro_z: float = 0.0
    
    # Battery
    battery_level: float = 100.0
    
    # Mode
    mode: int = 0  # 0: idle, 1: standing, 2: walking
    
    # Timestamp
    timestamp: float = 0.0


@dataclass 
class HighLevelCommand:
    """High-level velocity command for Go2"""
    # Linear velocity (m/s)
    vx: float = 0.0  # Forward/backward
    vy: float = 0.0  # Left/right (strafe)
    
    # Angular velocity (rad/s)
    wz: float = 0.0  # Yaw rotation
    
    # Body height adjustment (-1 to 1)
    body_height: float = 0.0
    
    # Gait type: 0=idle, 1=trot, 2=climb
    gait_type: int = 1
    
    # Mode: 0=idle, 1=force_stand, 2=walk
    mode: int = 2


class Go2HighLevelInterface:
    """
    High-level interface for Unitree Go2 robot
    
    This class handles UDP communication with the Go2's onboard computer
    for sending velocity commands and receiving state feedback.
    """
    
    # Default network configuration for Go2
    ROBOT_IP = "192.168.123.161"
    ROBOT_PORT = 8082
    LOCAL_PORT = 8081
    
    # Command frequency (Hz)
    CONTROL_FREQ = 50
    
    def __init__(
        self,
        robot_ip: str = ROBOT_IP,
        robot_port: int = ROBOT_PORT,
        local_port: int = LOCAL_PORT
    ):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.local_port = local_port
        
        self._socket: Optional[socket.socket] = None
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        
        self._current_state = RobotState()
        self._current_command = HighLevelCommand()
        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        
        self._state_callback: Optional[Callable[[RobotState], None]] = None
        self._last_command_time = 0.0
        self._command_timeout = 0.5  # seconds
        
    def connect(self) -> bool:
        """Establish UDP connection to robot"""
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind(('0.0.0.0', self.local_port))
            self._socket.settimeout(0.1)
            
            self._running = True
            self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._recv_thread.start()
            
            print(f"[Go2] Connected to robot at {self.robot_ip}:{self.robot_port}")
            return True
            
        except Exception as e:
            print(f"[Go2] Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close connection to robot"""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        if self._socket:
            self._socket.close()
            self._socket = None
        print("[Go2] Disconnected")
    
    def set_velocity(self, vx: float, vy: float, wz: float):
        """
        Set robot velocity command
        
        Args:
            vx: Forward velocity (m/s), positive = forward
            vy: Lateral velocity (m/s), positive = left
            wz: Angular velocity (rad/s), positive = counter-clockwise
        """
        with self._command_lock:
            self._current_command.vx = vx
            self._current_command.vy = vy
            self._current_command.wz = wz
            self._current_command.mode = 2 if (vx != 0 or vy != 0 or wz != 0) else 1
        self._send_command()
    
    def stop(self):
        """Stop the robot (zero velocity)"""
        self.set_velocity(0.0, 0.0, 0.0)
    
    def stand(self):
        """Command robot to stand in place"""
        with self._command_lock:
            self._current_command = HighLevelCommand(mode=1)
        self._send_command()
    
    def get_state(self) -> RobotState:
        """Get current robot state"""
        with self._state_lock:
            return RobotState(**vars(self._current_state))
    
    def set_state_callback(self, callback: Callable[[RobotState], None]):
        """Set callback for state updates"""
        self._state_callback = callback
    
    def _send_command(self):
        """Send command packet to robot"""
        if not self._socket:
            return
            
        with self._command_lock:
            cmd = self._current_command
            
        # Pack command into UDP packet
        # Format: mode(1B) + gait(1B) + vx(4B) + vy(4B) + wz(4B) + height(4B)
        packet = struct.pack(
            '<BBffff',
            cmd.mode,
            cmd.gait_type,
            cmd.vx,
            cmd.vy,
            cmd.wz,
            cmd.body_height
        )
        
        try:
            self._socket.sendto(packet, (self.robot_ip, self.robot_port))
            self._last_command_time = time.time()
        except Exception as e:
            print(f"[Go2] Send error: {e}")
    
    def _receive_loop(self):
        """Background thread for receiving state updates"""
        while self._running:
            try:
                data, addr = self._socket.recvfrom(1024)
                self._parse_state(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"[Go2] Receive error: {e}")
    
    def _parse_state(self, data: bytes):
        """Parse state packet from robot"""
        if len(data) < 80:  # Minimum expected packet size
            return
            
        try:
            # Unpack state data
            # Format matches typical Unitree high-level state
            values = struct.unpack('<3f4f3f3f3f3f1f1B1d', data[:80])
            
            with self._state_lock:
                self._current_state.x = values[0]
                self._current_state.y = values[1]
                self._current_state.z = values[2]
                self._current_state.qw = values[3]
                self._current_state.qx = values[4]
                self._current_state.qy = values[5]
                self._current_state.qz = values[6]
                self._current_state.vx = values[7]
                self._current_state.vy = values[8]
                self._current_state.vz = values[9]
                self._current_state.wx = values[10]
                self._current_state.wy = values[11]
                self._current_state.wz = values[12]
                self._current_state.imu_acc_x = values[13]
                self._current_state.imu_acc_y = values[14]
                self._current_state.imu_acc_z = values[15]
                self._current_state.imu_gyro_x = values[16]
                self._current_state.imu_gyro_y = values[17]
                self._current_state.imu_gyro_z = values[18]
                self._current_state.battery_level = values[19]
                self._current_state.mode = values[20]
                self._current_state.timestamp = values[21]
                
            if self._state_callback:
                self._state_callback(self._current_state)
                
        except Exception as e:
            print(f"[Go2] Parse error: {e}")


def euler_from_quaternion(qw: float, qx: float, qy: float, qz: float):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    
    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """
    Convert Euler angles to quaternion
    
    Returns:
        Tuple of (qw, qx, qy, qz)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return qw, qx, qy, qz


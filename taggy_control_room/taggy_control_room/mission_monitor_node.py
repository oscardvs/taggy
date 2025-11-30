#!/usr/bin/env python3
"""
Mission Monitor Node

Watches /mission/target_object topic and automatically launches
the autonomy/interception pipeline when a threat is detected.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import os
import threading


class MissionMonitorNode(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        
        # Parameters
        self.declare_parameter('launch_package', 'go2_tracker')
        self.declare_parameter('launch_file', 'tracker.launch.py')
        self.declare_parameter('auto_terminate_on_clear', True)
        
        self.launch_package = self.get_parameter('launch_package').value
        self.launch_file = self.get_parameter('launch_file').value
        self.auto_terminate = self.get_parameter('auto_terminate_on_clear').value
        
        # State
        self.autonomy_process = None
        self.current_target = None
        self.output_thread = None
        
        # Subscriber
        self.target_sub = self.create_subscription(
            String,
            '/mission/target_object',
            self._target_callback,
            10
        )
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("MISSION MONITOR NODE READY")
        self.get_logger().info(f"Watching: /mission/target_object")
        self.get_logger().info(f"Will launch: {self.launch_package} / {self.launch_file}")
        self.get_logger().info("=" * 50)
        
        self._publish_status("IDLE")
    
    def _target_callback(self, msg: String):
        target = msg.data.strip().lower()
        
        if not target:
            return
        
        # Handle clear/stop commands
        if target in ['clear', 'stop', 'cancel', 'abort']:
            self.get_logger().info(f"Received command: {target}")
            self._stop_autonomy()
            return
        
        # New target received
        if target != self.current_target:
            self.get_logger().info(f"New target received: {target}")
            
            # Stop existing autonomy if running
            if self.autonomy_process is not None:
                self.get_logger().info("Stopping current autonomy pipeline...")
                self._stop_autonomy()
            
            # Launch new autonomy pipeline
            self._launch_autonomy(target)
        else:
            self.get_logger().info(f"Target unchanged: {target}")
    
    def _log_output(self, process):
        """Thread function to log subprocess output"""
        try:
            for line in iter(process.stdout.readline, b''):
                if line:
                    self.get_logger().info(f"[TRACKER] {line.decode().strip()}")
        except Exception as e:
            self.get_logger().error(f"Output logging error: {e}")
    
    def _launch_autonomy(self, target: str):
        """Launch the autonomy/interception pipeline"""
        try:
            # Build command - note: argument name should match what tracker expects
            cmd = [
                'ros2', 'launch',
                self.launch_package,
                self.launch_file,
                f'threat_id:={target}'
            ]
            
            self.get_logger().info(f"Launching: {' '.join(cmd)}")
            
            # Set environment to ensure ROS is sourced
            env = os.environ.copy()
            
            # Launch as subprocess with output piping
            self.autonomy_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid
            )
            
            # Start thread to log output
            self.output_thread = threading.Thread(
                target=self._log_output,
                args=(self.autonomy_process,),
                daemon=True
            )
            self.output_thread.start()
            
            self.current_target = target
            self._publish_status(f"ACTIVE:{target}")
            
            self.get_logger().info(f"Autonomy pipeline launched (PID: {self.autonomy_process.pid})")
            
            # Check if process is still running after a short delay
            self.create_timer(2.0, self._check_process_status)
            
        except Exception as e:
            self.get_logger().error(f"Failed to launch autonomy: {e}")
            self._publish_status(f"ERROR:{e}")
    
    def _check_process_status(self):
        """Check if the launched process is still running"""
        if self.autonomy_process is not None:
            poll = self.autonomy_process.poll()
            if poll is not None:
                self.get_logger().error(f"Autonomy process exited with code: {poll}")
                self.autonomy_process = None
                self.current_target = None
                self._publish_status("FAILED")
            else:
                self.get_logger().info("Autonomy process running OK")
    
    def _stop_autonomy(self):
        """Stop the running autonomy pipeline"""
        if self.autonomy_process is not None:
            try:
                os.killpg(os.getpgid(self.autonomy_process.pid), signal.SIGTERM)
                self.autonomy_process.wait(timeout=5)
                self.get_logger().info("Autonomy pipeline stopped")
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.autonomy_process.pid), signal.SIGKILL)
                self.get_logger().warn("Autonomy pipeline force killed")
            except Exception as e:
                self.get_logger().error(f"Error stopping autonomy: {e}")
            finally:
                self.autonomy_process = None
                self.current_target = None
                self._publish_status("IDLE")
    
    def _publish_status(self, status: str):
        """Publish mission status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up on shutdown"""
        self._stop_autonomy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
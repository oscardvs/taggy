#!/usr/bin/env python3
"""
Keyboard Teleoperation for Unitree Go2
Control the robot manually using keyboard

Controls:
    Movement:
        W/S - Forward/Backward
        A/D - Strafe Left/Right
        Q/E - Rotate Left/Right
    
    Posture:
        1 - Stand Up
        2 - Sit Down
        3 - Balance Stand
        4 - Recovery Stand (get up from fallen)
        5 - Hello (wave)
    
    Speed:
        +/- - Increase/Decrease speed
    
    Other:
        SPACE - Stop movement
        X - Emergency Stop (disables motors!)
        R - Release emergency stop
        ESC - Quit

EDTH Hackathon Starter Pack by Laelaps AI
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import sys
import termios
import tty
import select


# Key mappings for movement
MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0),   # Forward
    's': (-1.0, 0.0, 0.0),  # Backward
    'a': (0.0, 1.0, 0.0),   # Strafe left
    'd': (0.0, -1.0, 0.0),  # Strafe right
    'q': (0.0, 0.0, 1.0),   # Rotate left (CCW)
    'e': (0.0, 0.0, -1.0),  # Rotate right (CW)
    'W': (1.0, 0.0, 0.0),
    'S': (-1.0, 0.0, 0.0),
    'A': (0.0, 1.0, 0.0),
    'D': (0.0, -1.0, 0.0),
    'Q': (0.0, 0.0, 1.0),
    'E': (0.0, 0.0, -1.0),
}

# Key mappings for posture commands
POSTURE_BINDINGS = {
    '1': 'up',        # Stand up
    '2': 'down',      # Sit/lie down
    '3': 'balance',   # Balance stand
    '4': 'recovery',  # Recovery stand
    '5': 'hello',     # Wave hello
    '6': 'stretch',   # Stretch
}

SPEED_BINDINGS = {
    '+': 1.1,   # Increase speed
    '=': 1.1,
    '-': 0.9,   # Decrease speed
    '_': 0.9,
}

HELP_TEXT = """
╔══════════════════════════════════════════════════════════════╗
║            UNITREE GO2 KEYBOARD TELEOPERATION                ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║   Movement:                     Rotation:                    ║
║       W                             Q    E                   ║
║     A   D                        (CCW)  (CW)                 ║
║       S                                                      ║
║                                                              ║
║   W/S - Forward/Backward        Q/E - Rotate Left/Right     ║
║   A/D - Strafe Left/Right                                   ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║   Posture Commands:                                          ║
║   1 - Stand Up          4 - Recovery (get up from fallen)   ║
║   2 - Sit Down          5 - Hello (wave)                    ║
║   3 - Balance Stand     6 - Stretch                         ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║   Speed Control:        +/- : Increase/Decrease speed       ║
║                                                              ║
║   Other:                                                     ║
║   SPACE : Stop movement                                     ║
║   X     : EMERGENCY STOP (disables motors!)                 ║
║   R     : Release emergency stop                            ║
║   ESC   : Quit                                              ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
"""


class KeyboardTeleop(Node):
    """Keyboard teleoperation node for Go2"""
    
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Publishers - use absolute topic names
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.posture_pub = self.create_publisher(String, '/cmd_posture', 10)
        
        # State
        self.estop_active = False
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Keyboard Teleop initialized')
    
    def get_key(self, timeout=0.1):
        """Get a single keypress with timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def send_posture(self, posture: str):
        """Send posture command"""
        msg = String()
        msg.data = posture
        self.posture_pub.publish(msg)
    
    def send_velocity(self, vx: float, vy: float, wz: float):
        """Send velocity command"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.cmd_vel_pub.publish(cmd)
    
    def send_estop(self, active: bool):
        """Send emergency stop command"""
        msg = Bool()
        msg.data = active
        self.estop_pub.publish(msg)
        self.estop_active = active
    
    def run(self):
        """Main teleop loop"""
        print(HELP_TEXT)
        print(f'\nCurrent speed: linear={self.linear_speed:.2f} m/s, angular={self.angular_speed:.2f} rad/s')
        print('\n>>> Press 1 to STAND UP the robot first! <<<')
        print('\nWaiting for keypresses...\n')
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if not key:
                    continue
                
                # ESC - Exit
                if key == '\x1b':
                    print('\nExiting... Stopping robot.')
                    self.send_velocity(0.0, 0.0, 0.0)
                    break
                
                # Emergency stop
                if key == 'x' or key == 'X':
                    print('\n' + '='*60)
                    print('⚠️  EMERGENCY STOP ACTIVATED - Motors disabled!')
                    print('    Press R to release and 1 to stand up again')
                    print('='*60)
                    self.send_estop(True)
                    self.send_velocity(0.0, 0.0, 0.0)
                    continue
                
                # Release emergency stop
                if key == 'r' or key == 'R':
                    if self.estop_active:
                        print('\n✓ Emergency stop released. Press 1 to stand up.')
                        self.send_estop(False)
                    continue
                
                # Posture commands
                if key in POSTURE_BINDINGS:
                    posture = POSTURE_BINDINGS[key]
                    print(f'\n>>> Posture: {posture.upper()}', end='\r')
                    self.send_posture(posture)
                    continue
                
                # Stop movement
                if key == ' ':
                    self.send_velocity(0.0, 0.0, 0.0)
                    print('STOP                                        ', end='\r')
                    continue
                
                # Speed adjustment
                if key in SPEED_BINDINGS:
                    factor = SPEED_BINDINGS[key]
                    self.linear_speed *= factor
                    self.angular_speed *= factor
                    self.linear_speed = max(0.1, min(1.5, self.linear_speed))
                    self.angular_speed = max(0.1, min(2.0, self.angular_speed))
                    print(f'Speed: linear={self.linear_speed:.2f} m/s, angular={self.angular_speed:.2f} rad/s', end='\r')
                    continue
                
                # Movement commands
                if key in MOVE_BINDINGS:
                    if self.estop_active:
                        print('⚠️  E-stop active! Press R to release, then 1 to stand.', end='\r')
                        continue
                    
                    vx, vy, wz = MOVE_BINDINGS[key]
                    vx *= self.linear_speed
                    vy *= self.linear_speed
                    wz *= self.angular_speed
                    
                    self.send_velocity(vx, vy, wz)
                    print(f'vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}                ', end='\r')
                
        except Exception as e:
            print(f'\nError: {e}')
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            # Stop the robot
            self.send_velocity(0.0, 0.0, 0.0)
    
    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

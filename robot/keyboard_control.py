#!/usr/bin/env python3
"""
Robot keyboard control using arrow keys
Controls a differential drive robot via MotorDriver class
"""

import sys
import tty
import termios
from robot.motor_driver import MotorDriver

# Control parameters
SPEED_NORMAL = 60      # Normal forward/backward speed (%)
SPEED_TURN = 50        # Speed for turning (%)
SPEED_ROTATE = 40      # Speed for in-place rotation (%)

class KeyboardControl:
    def __init__(self):
        self.driver = MotorDriver(v_max=200)  # Initialize motor driver
        self.running = True
        
    def get_key(self):
        """Get a single keypress from the terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            
            # Handle arrow keys (they send escape sequences)
            if ch == '\x1b':  # Escape sequence
                ch = sys.stdin.read(2)
                if ch == '[A':
                    return 'UP'
                elif ch == '[B':
                    return 'DOWN'
                elif ch == '[C':
                    return 'RIGHT'
                elif ch == '[D':
                    return 'LEFT'
            return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*50)
        print("ROBOT KEYBOARD CONTROL")
        print("="*50)
        print("\nControls:")
        print("  ↑ (UP)    - Move forward")
        print("  ↓ (DOWN)  - Move backward")
        print("  ← (LEFT)  - Turn left")
        print("  → (RIGHT) - Turn right")
        print("  W         - Forward")
        print("  S         - Backward")
        print("  A         - Rotate left (in place)")
        print("  D         - Rotate right (in place)")
        print("  SPACE     - Stop")
        print("  Q or ESC  - Quit")
        print("\nPress any key to start...\n")
    
    def handle_key(self, key):
        """Process keyboard input and control motors"""
        
        if key == 'UP' or key == 'w' or key == 'W':
            # Forward
            print("Forward   ", end='\r')
            self.driver.set_wheel_duty(SPEED_NORMAL, SPEED_NORMAL)
            
        elif key == 'DOWN' or key == 's' or key == 'S':
            # Backward
            print("Backward  ", end='\r')
            self.driver.set_wheel_duty(-SPEED_NORMAL, -SPEED_NORMAL)
            
        elif key == 'LEFT':
            # Turn left (left wheel slower)
            print("Turn Left ", end='\r')
            self.driver.set_wheel_duty(SPEED_TURN * 0.3, SPEED_TURN)
            
        elif key == 'RIGHT':
            # Turn right (right wheel slower)
            print("Turn Right", end='\r')
            self.driver.set_wheel_duty(SPEED_TURN, SPEED_TURN * 0.3)
            
        elif key == 'a' or key == 'A':
            # Rotate left in place
            print("Rotate Left ", end='\r')
            self.driver.set_wheel_duty(-SPEED_ROTATE, SPEED_ROTATE)
            
        elif key == 'd' or key == 'D':
            # Rotate right in place
            print("Rotate Right", end='\r')
            self.driver.set_wheel_duty(SPEED_ROTATE, -SPEED_ROTATE)
            
        elif key == ' ':
            # Stop
            print("Stop      ", end='\r')
            self.driver.stop()
            
        elif key == 'q' or key == 'Q' or key == '\x1b':
            # Quit
            print("\nQuitting...")
            self.running = False
            self.driver.stop()
            
        else:
            # Unknown key - stop for safety
            self.driver.stop()
    
    def run(self):
        """Main control loop"""
        self.print_instructions()
        
        # Wait for initial keypress
        self.get_key()
        
        print("\n🤖 Robot control active! Use arrow keys to drive.\n")
        
        try:
            while self.running:
                key = self.get_key()
                self.handle_key(key)
                
        except KeyboardInterrupt:
            print("\n\nInterrupted by user (Ctrl+C)")
            
        except Exception as e:
            print("\n\nError: ", e)
            
        finally:
            print("\nStopping motors and cleaning up...")
            self.driver.stop()
            self.driver.cleanup()
            print("Done!")



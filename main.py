
from robot.keyboard_control import KeyboardControl


if __name__ == "__main__":
    print("Starting robot keyboard control...")
    print("Make sure your robot is on blocks or in a safe area!")
    
    controller = KeyboardControl()
    controller.run()
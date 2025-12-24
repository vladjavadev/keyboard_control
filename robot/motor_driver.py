import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import math
from enum import Enum

class DIR(Enum):
    FORWARD = 1
    STOP = 0
    REVERSE = -1


class MotorDriver:
    """Real MotorDriver for BeagleBone Black hardware with TB6612FNG driver"""
    
    # PWM pins
    pin_PWMA = "P9_21"  # PWM for Motor A (left motor)
    pin_PWMB = "P9_22"  # PWM for Motor B (right motor)
    
    # Motor A (left) direction control pins
    pin_AIN1 = "P9_27"  # Motor A input 1
    pin_AIN2 = "P9_29"  # Motor A input 2
    
    # Motor B (right) direction control pins
    pin_BIN1 = "P9_25"  # Motor B input 1
    pin_BIN2 = "P9_23"  # Motor B input 2

    FREQ = 25000  # 25kHz PWM frequency (typical for TB6612FNG)
    pwm_max = 90
    pwm_min = 30

    def __init__(self, v_max):
        """Initialize motor driver with hardware PWM
        
        Args:
            v_max: Maximum velocity in mm/s
        """
        # Setup direction pins as outputs
        GPIO.setup(self.pin_AIN1, GPIO.OUT)
        GPIO.setup(self.pin_AIN2, GPIO.OUT)
        GPIO.setup(self.pin_BIN1, GPIO.OUT)
        GPIO.setup(self.pin_BIN2, GPIO.OUT)
        
        # Initialize direction pins to LOW (stopped)
        GPIO.output(self.pin_AIN1, GPIO.LOW)
        GPIO.output(self.pin_AIN2, GPIO.LOW)
        GPIO.output(self.pin_BIN1, GPIO.LOW)
        GPIO.output(self.pin_BIN2, GPIO.LOW)
        
        # Start PWM channels at 0% duty cycle
        PWM.start(self.pin_PWMA, 0, self.FREQ, 0)
        PWM.start(self.pin_PWMB, 0, self.FREQ, 0)
        
        self.v_max = v_max
        self.pwm_l = 0
        self.pwm_r = 0

    def set_wheel_duty(self, duty_l, duty_r):
        """Set wheel duty cycle from PID controller output
        
        Args:
            duty_l: Left wheel duty cycle (-100 to 100)
            duty_r: Right wheel duty cycle (-100 to 100)
        """
        # Clamp input values to valid range
        duty_l = max(-100, min(100, duty_l))
        duty_r = max(-100, min(100, duty_r))
        
        # Calculate PWM values (map from 0-100% to pwm_min-pwm_max range)
        if abs(duty_l) > 0:
            self.pwm_l = int(self.pwm_min + (abs(duty_l) / 100.0) * (self.pwm_max - self.pwm_min))
        else:
            self.pwm_l = 0
            
        if abs(duty_r) > 0:
            self.pwm_r = int(self.pwm_min + (abs(duty_r) / 100.0) * (self.pwm_max - self.pwm_min))
        else:
            self.pwm_r = 0

        # Left motor (Motor A) control
        # Direction: AIN1=HIGH, AIN2=LOW -> Forward
        #           AIN1=LOW, AIN2=HIGH -> Reverse
        #           AIN1=LOW, AIN2=LOW  -> Stop (brake)
        if duty_l > 0:  # Forward
            GPIO.output(self.pin_AIN1, GPIO.HIGH)
            GPIO.output(self.pin_AIN2, GPIO.LOW)
            PWM.set_duty_cycle(self.pin_PWMA, self.pwm_l)
        elif duty_l < 0:  # Reverse
            GPIO.output(self.pin_AIN1, GPIO.LOW)
            GPIO.output(self.pin_AIN2, GPIO.HIGH)
            PWM.set_duty_cycle(self.pin_PWMA, self.pwm_l)
        else:  # Stop
            GPIO.output(self.pin_AIN1, GPIO.LOW)
            GPIO.output(self.pin_AIN2, GPIO.LOW)
            PWM.set_duty_cycle(self.pin_PWMA, 0)

        # Right motor (Motor B) control
        # Direction: BIN1=HIGH, BIN2=LOW -> Forward
        #           BIN1=LOW, BIN2=HIGH -> Reverse
        #           BIN1=LOW, BIN2=LOW  -> Stop (brake)
        if duty_r > 0:  # Forward
            GPIO.output(self.pin_BIN1, GPIO.HIGH)
            GPIO.output(self.pin_BIN2, GPIO.LOW)
            PWM.set_duty_cycle(self.pin_PWMB, self.pwm_r)
        elif duty_r < 0:  # Reverse
            GPIO.output(self.pin_BIN1, GPIO.LOW)
            GPIO.output(self.pin_BIN2, GPIO.HIGH)
            PWM.set_duty_cycle(self.pin_PWMB, self.pwm_r)
        else:  # Stop
            GPIO.output(self.pin_BIN1, GPIO.LOW)
            GPIO.output(self.pin_BIN2, GPIO.LOW)
            PWM.set_duty_cycle(self.pin_PWMB, 0)

    def set_motor_speed(self, vl, vr):
        """Set motor speed in mm/s (legacy method for compatibility)
        
        Args:
            vl: Left wheel velocity in mm/s
            vr: Right wheel velocity in mm/s
        """
        # Convert velocity to duty cycle percentage
        if self.v_max == 0:
            duty_l = 0
            duty_r = 0
        else:
            duty_l = (vl / self.v_max) * 100.0
            duty_r = (vr / self.v_max) * 100.0
        
        # Use the main control method
        self.set_wheel_duty(duty_l, duty_r)

    def stop(self):
        """Emergency stop - set all motors to brake mode"""
        # Set all direction pins LOW for brake
        GPIO.output(self.pin_AIN1, GPIO.LOW)
        GPIO.output(self.pin_AIN2, GPIO.LOW)
        GPIO.output(self.pin_BIN1, GPIO.LOW)
        GPIO.output(self.pin_BIN2, GPIO.LOW)
        
        # Set PWM to 0
        PWM.set_duty_cycle(self.pin_PWMA, 0)
        PWM.set_duty_cycle(self.pin_PWMB, 0)
        
        self.pwm_l = 0
        self.pwm_r = 0

    def cleanup(self):
        """Clean up PWM and GPIO resources"""
        self.stop()
        PWM.stop(self.pin_PWMA)
        PWM.stop(self.pin_PWMB)
        PWM.cleanup()
        GPIO.cleanup()


# Test code for hardware verification
if __name__ == "__main__":
    import time
    
    print("=== Testing MotorDriver on BeagleBone with TB6612FNG ===")
    print("Pin Configuration:")
    print("  PWMA (P8_21) - Left Motor Speed")
    print("  PWMB (P8_22) - Right Motor Speed")
    print("  AIN1 (P8_27), AIN2 (P8_29) - Left Motor Direction")
    print("  BIN1 (P8_25), BIN2 (P8_23) - Right Motor Direction")
    print("\nWARNING: Motors will run! Make sure robot is on blocks or safely secured.\n")
    
    try:
        driver = MotorDriver(v_max=200)
        
        # Test 1: Forward motion at 50%
        print("Test 1: Forward motion (50% duty) for 2 seconds")
        driver.set_wheel_duty(50, 50)
        time.sleep(2)
        
        # Test 2: Stop
        print("Test 2: Stop for 1 second")
        driver.stop()
        time.sleep(1)
        
        # Test 3: Turn left
        print("Test 3: Turn left (30% left, 70% right) for 2 seconds")
        driver.set_wheel_duty(30, 70)
        time.sleep(2)
        
        # Test 4: Stop
        print("Test 4: Stop for 1 second")
        driver.stop()
        time.sleep(1)
        
        # Test 5: Reverse
        print("Test 5: Reverse (-50% duty) for 2 seconds")
        driver.set_wheel_duty(-50, -50)
        time.sleep(2)
        
        # Test 6: Stop
        print("Test 6: Stop for 1 second")
        driver.stop()
        time.sleep(1)
        
        # Test 7: Rotate in place
        print("Test 7: Rotate in place for 2 seconds")
        driver.set_wheel_duty(-50, 50)
        time.sleep(2)
        
        # Final stop
        print("Test complete - stopping motors")
        driver.stop()
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        driver.stop()
    except Exception as e:
        print("\n\nError during test: ", e)
        driver.stop()
    finally:
        print("Cleaning up resources")
        driver.cleanup()
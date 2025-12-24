#!/usr/bin/env python3
"""
Adafruit_BBIO compatible wrapper using libgpiod v2.x (new API)
Drop-in replacement for Adafruit_BBIO.GPIO and Adafruit_BBIO.PWM
Optimized for BeagleBone Green Wireless
"""

import gpiod
from gpiod.line import Direction, Value, Bias
import threading
import time

# BeagleBone Green/Green Wireless pin to GPIO mapping
# GPIO number formula: (gpiochip * 32) + line_offset
PIN_MAP = {
    # P8 Header
    "P8_3": 38,   "P8_4": 39,   "P8_5": 34,   "P8_6": 35,
    "P8_7": 66,   "P8_8": 67,   "P8_9": 69,   "P8_10": 68,
    "P8_11": 45,  "P8_12": 44,  "P8_13": 23,  "P8_14": 26,
    "P8_15": 47,  "P8_16": 46,  "P8_17": 27,  "P8_18": 65,
    "P8_19": 22,  "P8_20": 63,  "P8_21": 62,  "P8_22": 37,
    "P8_23": 36,  "P8_24": 33,  "P8_25": 32,  "P8_26": 61,
    "P8_27": 86,  "P8_28": 88,  "P8_29": 87,  "P8_30": 89,
    "P8_31": 10,  "P8_32": 11,  "P8_33": 9,   "P8_34": 81,
    "P8_35": 8,   "P8_36": 80,  "P8_37": 78,  "P8_38": 79,
    "P8_39": 76,  "P8_40": 77,  "P8_41": 74,  "P8_42": 75,
    "P8_43": 72,  "P8_44": 73,  "P8_45": 70,  "P8_46": 71,
    
    # P9 Header
    "P9_11": 126, "P9_12": 60,  "P9_13": 31,  "P9_14": 50,
    "P9_15": 48,  "P9_16": 51,  "P9_17": 5,   "P9_18": 4,
    "P9_19": 13,  "P9_20": 12,  "P9_21": 3,   "P9_22": 2,
    "P9_23": 49,  "P9_24": 15,  "P9_25": 117, "P9_26": 14,
    "P9_27": 115, "P9_28": 113, "P9_29": 111, "P9_30": 112,
    "P9_31": 110, "P9_41": 20,  "P9_42": 7,
}


class GPIO:
    """GPIO wrapper compatible with Adafruit_BBIO.GPIO"""
    
    # Constants matching Adafruit_BBIO
    OUT = "OUT"
    IN = "IN"
    HIGH = 1
    LOW = 0
    PUD_UP = 1
    PUD_DOWN = 2
    PUD_OFF = 0
    
    # Internal state
    _line_requests = {}
    _pin_info = {}
    
    @classmethod
    def _pin_to_gpio(cls, pin):
        """Convert BeagleBone pin name to (chip_path, line_offset) tuple
        
        BeagleBone GPIO numbering: GPIO_number = (chip * 32) + line_offset
        Example: GPIO 126 = gpiochip3 * 32 + line 30 = 96 + 30
        """
        if isinstance(pin, tuple):
            return pin
        if pin not in PIN_MAP:
            raise ValueError(f"Invalid pin: {pin}")
        
        gpio_num = PIN_MAP[pin]
        
        # Calculate chip and line from global GPIO number
        chip_num = gpio_num // 32
        line_offset = gpio_num % 32
        chip_path = f"/dev/gpiochip{chip_num}"
        
        return (chip_path, line_offset)
    
    @classmethod
    def setup(cls, pin, direction, pull_up_down=PUD_OFF, initial=LOW):
        """Setup a GPIO pin
        
        Args:
            pin: Pin name (e.g., "P9_11") or GPIO number
            direction: GPIO.OUT or GPIO.IN
            pull_up_down: Pull up/down resistor
            initial: Initial value for output pins
        """
        chip_path, line_offset = cls._pin_to_gpio(pin)
        
        # Release line if already setup
        if pin in cls._line_requests:
            cls._line_requests[pin].release()
            del cls._line_requests[pin]
        
        # Store chip and offset for this pin
        cls._pin_info[pin] = (chip_path, line_offset)
        
        # Configure line settings
        if direction == cls.OUT:
            line_settings = {
                line_offset: gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.ACTIVE if initial == cls.HIGH else Value.INACTIVE
                )
            }
        else:
            bias = Bias.DISABLED
            if pull_up_down == cls.PUD_UP:
                bias = Bias.PULL_UP
            elif pull_up_down == cls.PUD_DOWN:
                bias = Bias.PULL_DOWN
            
            line_settings = {
                line_offset: gpiod.LineSettings(
                    direction=Direction.INPUT,
                    bias=bias
                )
            }
        
        # Request line
        request = gpiod.request_lines(
            chip_path,
            config=line_settings,
            consumer="bbio_wrapper"
        )
        
        cls._line_requests[pin] = request
    
    @classmethod
    def output(cls, pin, value):
        """Set output pin value"""
        if pin not in cls._line_requests:
            raise RuntimeError(f"Pin {pin} not setup. Call GPIO.setup() first.")
        
        chip_path, line_offset = cls._pin_info[pin]
        val = Value.ACTIVE if value == cls.HIGH else Value.INACTIVE
        cls._line_requests[pin].set_value(line_offset, val)
    
    @classmethod
    def input(cls, pin):
        """Read input pin value"""
        if pin not in cls._line_requests:
            raise RuntimeError(f"Pin {pin} not setup. Call GPIO.setup() first.")
        
        chip_path, line_offset = cls._pin_info[pin]
        val = cls._line_requests[pin].get_value(line_offset)
        return cls.HIGH if val == Value.ACTIVE else cls.LOW
    
    @classmethod
    def cleanup(cls, pin=None):
        """Cleanup GPIO resources"""
        if pin is None:
            for request in cls._line_requests.values():
                try:
                    request.release()
                except:
                    pass
            cls._line_requests.clear()
            cls._pin_info.clear()
        else:
            if pin in cls._line_requests:
                try:
                    cls._line_requests[pin].release()
                except:
                    pass
                del cls._line_requests[pin]
            if pin in cls._pin_info:
                del cls._pin_info[pin]


class PWM:
    """PWM wrapper compatible with Adafruit_BBIO.PWM using software PWM"""
    
    _pwm_threads = {}
    _pwm_states = {}
    
    @classmethod
    def start(cls, pin, duty_cycle=0, frequency=2000, polarity=0):
        """Start PWM on a pin"""
        # Setup pin as output if not already done
        if pin not in GPIO._line_requests:
            GPIO.setup(pin, GPIO.OUT)
        
        # Stop existing PWM if running
        if pin in cls._pwm_threads:
            cls.stop(pin)
        
        # Create PWM state
        state = {
            'running': True,
            'duty_cycle': duty_cycle,
            'frequency': frequency,
            'polarity': polarity,
            'pin': pin
        }
        cls._pwm_states[pin] = state
        
        # Start PWM thread
        thread = threading.Thread(target=cls._pwm_worker, args=(pin, state), daemon=True)
        thread.start()
        cls._pwm_threads[pin] = thread
    
    @classmethod
    def _pwm_worker(cls, pin, state):
        """Software PWM worker thread"""
        while state['running']:
            try:
                if state['duty_cycle'] <= 0:
                    val = 0 if state['polarity'] == 0 else 1
                    GPIO.output(pin, val)
                    time.sleep(0.01)
                elif state['duty_cycle'] >= 100:
                    val = 1 if state['polarity'] == 0 else 0
                    GPIO.output(pin, val)
                    time.sleep(0.01)
                else:
                    period = 1.0 / state['frequency']
                    high_time = (state['duty_cycle'] / 100.0) * period
                    low_time = period - high_time
                    
                    val = 1 if state['polarity'] == 0 else 0
                    GPIO.output(pin, val)
                    time.sleep(high_time)
                    
                    val = 0 if state['polarity'] == 0 else 1
                    GPIO.output(pin, val)
                    time.sleep(low_time)
            except:
                break
    
    @classmethod
    def stop(cls, pin):
        """Stop PWM on a pin"""
        if pin in cls._pwm_states:
            cls._pwm_states[pin]['running'] = False
            
        if pin in cls._pwm_threads:
            cls._pwm_threads[pin].join(timeout=0.1)
            del cls._pwm_threads[pin]
            
        if pin in cls._pwm_states:
            del cls._pwm_states[pin]
        
        try:
            GPIO.output(pin, GPIO.LOW)
        except:
            pass
    
    @classmethod
    def set_duty_cycle(cls, pin, duty_cycle):
        """Set PWM duty cycle (0-100)"""
        if pin not in cls._pwm_states:
            raise RuntimeError(f"PWM not started on pin {pin}. Call PWM.start() first.")
        cls._pwm_states[pin]['duty_cycle'] = max(0, min(100, duty_cycle))
    
    @classmethod
    def set_frequency(cls, pin, frequency):
        """Set PWM frequency in Hz"""
        if pin not in cls._pwm_states:
            raise RuntimeError(f"PWM not started on pin {pin}. Call PWM.start() first.")
        cls._pwm_states[pin]['frequency'] = frequency
    
    @classmethod
    def cleanup(cls):
        """Cleanup all PWM resources"""
        pins = list(cls._pwm_threads.keys())
        for pin in pins:
            cls.stop(pin)


# Test code
if __name__ == "__main__":
    print("=== Testing Adafruit_BBIO wrapper ===")
    print("BeagleBone Green Wireless\n")
    
    try:
        # Test P9_11 (gpiochip3, line 30)
        print("Testing P9_11:")
        chip_path, line = GPIO._pin_to_gpio("P9_11")
        print(f"  Mapped to: {chip_path}, line {line}")
        
        GPIO.setup("P9_11", GPIO.OUT)
        print("  Setup complete")
        
        for i in range(10):
            GPIO.output("P9_11", GPIO.HIGH)
            print(f"  [{i+1}/10] HIGH")
            time.sleep(2.0)
            
            GPIO.output("P9_11", GPIO.LOW)
            print(f"  [{i+1}/10] LOW")
            time.sleep(2.0)
        
        for i in range(5):
            duty = i * 20
            print(f"  Setting PWM duty cycle to {duty}%")
            PWM.start("P9_21", duty_cycle=duty, frequency=1000)
            time.sleep(2.0)
            PWM.stop("P9_21")
        print("\n✓ Test complete!")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        PWM.cleanup()
        GPIO.cleanup()
        print("Done!")
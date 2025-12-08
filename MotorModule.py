import lgpio
from time import sleep

class Motor():
    def __init__(self, EnaA, In1A, In2A, EnaB, In1B, In2B):
        self.EnaA = EnaA
        self.In1A = In1A
        self.In2A = In2A
        self.EnaB = EnaB
        self.In1B = In1B
        self.In2B = In2B
        
        # Open GPIO chip
        self.h = lgpio.gpiochip_open(0)
        
        # Set all pins as outputs
        lgpio.gpio_claim_output(self.h, self.EnaA,0)
        lgpio.gpio_claim_output(self.h, self.In1A,0)
        lgpio.gpio_claim_output(self.h, self.In2A,0)
        lgpio.gpio_claim_output(self.h, self.EnaB,0)
        lgpio.gpio_claim_output(self.h, self.In1B,0)
        lgpio.gpio_claim_output(self.h, self.In2B,0)
        
        # Initialize PWM (frequency = 100Hz)
        self.pwm_freq = 100
        lgpio.tx_pwm(self.h, self.EnaA, self.pwm_freq, 0)
        lgpio.tx_pwm(self.h, self.EnaB, self.pwm_freq, 0)
    
    def move(self, speed=0.5, turn=0, t=None):
        speed *= 50
        turn *= 50

        leftSpeed = speed - turn
        rightSpeed = speed + turn

        if leftSpeed > 100:
            leftSpeed = 100
        elif leftSpeed < -100:
            leftSpeed = -100

        if rightSpeed > 100:
            rightSpeed = 100
        elif rightSpeed < -100:
            rightSpeed = -100

        # PWM
        lgpio.tx_pwm(self.h, self.EnaA, self.pwm_freq, abs(rightSpeed))
        lgpio.tx_pwm(self.h, self.EnaB, self.pwm_freq, abs(leftSpeed))

        # Right motor (A)
        if leftSpeed > 0:
            lgpio.gpio_write(self.h, self.In1A, 1)
            lgpio.gpio_write(self.h, self.In2A, 0)
        else:
            lgpio.gpio_write(self.h, self.In1A, 0)
            lgpio.gpio_write(self.h, self.In2A, 1)

        # Left motor (B)
        if rightSpeed > 0:
            lgpio.gpio_write(self.h, self.In1B, 1)
            lgpio.gpio_write(self.h, self.In2B, 0)
        else:
            lgpio.gpio_write(self.h, self.In1B, 0)
            lgpio.gpio_write(self.h, self.In2B, 1)

        if t is not None and t > 0:
            sleep(t)
            self.stop()

    
    def stop(self, t=0):
        lgpio.tx_pwm(self.h, self.EnaA, self.pwm_freq, 0)
        lgpio.tx_pwm(self.h, self.EnaB, self.pwm_freq, 0)
        if t > 0:
            sleep(t)
    
    def move_left(self, speed=20):
        lgpio.tx_pwm(self.h, self.EnaA, self.pwm_freq, speed)
        lgpio.tx_pwm(self.h, self.EnaB, self.pwm_freq, speed)
        lgpio.gpio_write(self.h, self.In1A, 1)
        lgpio.gpio_write(self.h, self.In2A, 0)
        lgpio.gpio_write(self.h, self.In1B, 0)
        lgpio.gpio_write(self.h, self.In2B, 1)
    
    def move_straight(self, speed=20):
        lgpio.tx_pwm(self.h, self.EnaA, self.pwm_freq, speed)
        lgpio.tx_pwm(self.h, self.EnaB, self.pwm_freq, speed)
        lgpio.gpio_write(self.h, self.In1A, 1)
        lgpio.gpio_write(self.h, self.In2A, 0)
        lgpio.gpio_write(self.h, self.In1B, 1)
        lgpio.gpio_write(self.h, self.In2B, 0)
    
    def move_right(self, speed=20):
        lgpio.tx_pwm(self.h, self.EnaA, self.pwm_freq, speed)
        lgpio.tx_pwm(self.h, self.EnaB, self.pwm_freq, speed)
        lgpio.gpio_write(self.h, self.In1A, 0)
        lgpio.gpio_write(self.h, self.In2A, 1)
        lgpio.gpio_write(self.h, self.In1B, 1)
        lgpio.gpio_write(self.h, self.In2B, 0)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        lgpio.gpiochip_close(self.h)


if __name__ == "__main__":
    # Initialize motor with your pin numbers
    motor = Motor(EnaA=2, In1A=3, In2A=4, EnaB=17, In1B=22, In2B=27)
    
    try:
        motor.move_straight()
        sleep(5)
        motor.stop()
    finally:
        motor.cleanup()
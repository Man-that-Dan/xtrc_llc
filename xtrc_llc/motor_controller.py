#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from time import sleep          


class MotorController(Node): 
    def __init__(self):
        super().__init__("motor_controller") 
        self.subscriber_ = self.create_subscription(
            Twist, "/cmd_vel", self.operate_motors, 10)
        self.get_logger().info("motor controller init")

        #mins and parameters
        self.drive_in1 = 5
        self.drive_in2 = 4
        self.steer_in1 = 3
        self.steer_in2 = 2
        self.drive_en = 19
        self.steer_en = 13
        self.range = 90
        self.max_duty_cycle = 100
        self.min_effective_duty_cycle = 45
        self.min_duty_cycle = 6

        #setup pins for driving motor
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.drive_in1,GPIO.OUT)
        GPIO.setup(self.drive_in2,GPIO.OUT)
        GPIO.setup(self.drive_en,GPIO.OUT)
        GPIO.output(self.drive_in1,GPIO.LOW)
        GPIO.output(self.drive_in2,GPIO.LOW)

        #set frequency to 1000Hz
        self.drive=GPIO.PWM(self.drive_en,1000)

        #setup pins for driving motor
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.steer_in1,GPIO.OUT)
        GPIO.setup(self.steer_in2,GPIO.OUT)
        GPIO.setup(self.steer_en,GPIO.OUT)
        GPIO.output(self.steer_in1,GPIO.LOW)
        GPIO.output(self.steer_in2,GPIO.LOW)

        #set frequency to 1000Hz
        self.steer=GPIO.PWM(self.steer_en,1000)

        self.drive.start(25)
        self.steer.start(25)

    #note: have dc motors without encoders so I'm not quite sure what actual corresponds to what duty cycle and I don't have a stroboscope at the moment
    # so velocity here is just a range of possible output
    # this particular model also has a brushed motor controlling the steering instead of a servo so I'm approximating steering with that too
    def operate_motors(self, msg):
        self.get_logger().debug(msg)
        self.dir = 1
        self.velocity = msg.linear.x * self.max_duty_cycle
        if self.velocity < 0:
            self.dir = -1
            self.velocity = self.velocity * -1

        steering_dir = 1
        steering_angle = msg.angular.z * self.max_duty_cycle
        if steering_angle < 0:
            steering_dir = -1
            steering_angle = steering_angle * -1
        if steering_angle > 100:
            steering_angle = 100
        self.steer.ChangeDutyCycle(steering_angle)

        #set steering direction
        if steering_angle == 0:
            GPIO.output(self.steer_in1,GPIO.LOW)
            GPIO.output(self.steer_in2,GPIO.LOW)
        elif steering_dir < 0:
            GPIO.output(self.steer_in1,GPIO.LOW)
            GPIO.output(self.steer_in2,GPIO.HIGH)
        else:
            GPIO.output(self.steer_in1,GPIO.HIGH)
            GPIO.output(self.steer_in2,GPIO.LOW)

        #motor just cannot spin below this duty cycle so just stop
        if self.velocity < self.min_duty_cycle:
            self.velocity = 0

        drive_1 = GPIO.input(self.drive_in1)
        drive_2 = GPIO.input(self.drive_in2)

        if self.dir > 0 and self.velocity > 0.1:
            #move forward
            GPIO.output(self.drive_in1,GPIO.LOW)
            GPIO.output(self.drive_in2,GPIO.HIGH)
        elif self.velocity < 0.1:
            #stop
            GPIO.output(self.drive_in1,GPIO.LOW)
            GPIO.output(self.drive_in2,GPIO.LOW)
        else:
            #reverse
            GPIO.output(self.drive_in1,GPIO.HIGH)
            GPIO.output(self.drive_in2,GPIO.LOW)

        drive_1_new = GPIO.input(self.drive_in1)
        drive_2_new = GPIO.input(self.drive_in2)

        #if changing directions or starting up and speed a little low give a little push
        if self.velocity < self.min_effective_duty_cycle and self.velocity > 0:
            if drive_1 != drive_1_new or drive_2 != drive_2_new:
                self.drive.ChangeDutyCycle(self.max_duty_cycle)
                sleep(0.25)
                self.drive.ChangeDutyCycle(self.velocity)
            else:
                self.drive.ChangeDutyCycle(self.velocity)
        else:
            #else just set speed
            self.drive.ChangeDutyCycle(self.velocity)
            
def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

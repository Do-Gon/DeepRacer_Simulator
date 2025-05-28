import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import sys
import select
import tty
import termios

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(ServoCtrlMsg, 'cmd_servo_control', 10)
        # Assuming a typical scaling for the angle from -30 to 30 degrees and throttle from -1 to 1
        self.max_throttle = 1.0  # Max throttle value (ratio or PWM)
        self.max_steering = 30.0  # Max steering angle (ratio or PWM)
        self.throttle_step = 0.05
        self.steering_step = 0.1  # Smaller step for finer control

        self.throttle = 0.0
        self.steering = 0.0

        self.msg = """
Control Your DeepRacer!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease throttle (scale or PWM)
a/d : increase/decrease steering angle (scale or PWM)

s : force stop

CTRL-C to quit
"""

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def constrain(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def update_command(self, key):
        if key == 'w':
            self.throttle = self.constrain(self.throttle + self.throttle_step, -self.max_throttle, self.max_throttle)
        elif key == 's':
            self.throttle = 0
            self.steering = 0
        elif key == 'x':
            self.throttle = self.constrain(self.throttle - self.throttle_step, -self.max_throttle, self.max_throttle)
        elif key == 'a':
            self.steering = self.constrain(self.steering + self.steering_step, -self.max_steering, self.max_steering)
        elif key == 'd':
            self.steering = self.constrain(self.steering - self.steering_step, -self.max_steering, self.max_steering)
        elif key == '\x03':
            raise KeyboardInterrupt()
        
        # Print the current values of throttle and steering
        print(f"Current throttle: {self.throttle:.2f}, Current steering: {self.steering:.2f}")

    def publish_command(self):
        msg = ServoCtrlMsg()
        msg.angle = float(self.steering)
        msg.throttle = float(self.throttle)
        self.publisher_.publish(msg)

    def run(self):
        print(self.msg)
        self.settings = termios.tcgetattr(sys.stdin)
        try:
            while True:
                key = self.get_key()
                self.update_command(key)
                self.publish_command()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_keyboard = TeleopTwistKeyboard()
    teleop_twist_keyboard.run()
    teleop_twist_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

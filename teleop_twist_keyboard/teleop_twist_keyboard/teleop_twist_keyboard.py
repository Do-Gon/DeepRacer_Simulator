import sys
import threading

import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg  # Correct import path

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as ServoCtrl messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

def main():
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(ServoCtrlMsg, 'cmd_servo_control', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5  # This represents the throttle ratio
    turn = 0.2   # This represents the angle ratio

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
            elif key in speedBindings.keys():
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(vels(speed, turn))
            else:
                x, y, z, th = 0, 0, 0, 0
                if key == '\x03':
                    break

            servo_msg = ServoCtrlMsg()
            servo_msg.throttle = x * speed  # Set throttle ratio
            servo_msg.angle = th * turn     # Set angle ratio
            pub.publish(servo_msg)


    except Exception as e:
        print(e)

    finally:
        pub.publish(ServoCtrlMsg())  # Send a stop message to ensure robot stops if the program exits
        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
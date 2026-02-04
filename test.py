from pynput import keyboard
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles

class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)

    def send_msg_stop(self):
        msg = DutyCycles()
        msg.duty_cycle_left = 0
        msg.duty_cycle_right = 0
        self.pub.publish(msg)

    def on_press(key, self):
        #print(f'Key {key.char} pressed')
        try:
            if key.char == 'm':
                self.send_msg_stop()
        except AttributeError:
            pass
            

def main():
    rclpy.init()
    node = Driver()

    listener = keyboard.Listener(
        on_press=node.on_press)#,
        #on_release=on_release)
    listener.start() #on a seperate thread (non-blocking)

    # while(True):
    #      continue

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


# def on_release(key):
#     print(f'Key {key.char} released')
#     if key == keyboard.Key.esc:
#         # Stop listener
#         return False

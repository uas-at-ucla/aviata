import rclpy
import sys
import os
from rclpy.node import Node
from std_msgs.msg import String

sys.path.append(os.path.abspath('../controls/build/rosidl_generator_py')) # path dependent
from aviata.msg import DroneStatus

os.environ['LD_LIBRARY_PATH'] = os.environ['LD_LIBRARY_PATH'] + ":" + os.path.abspath("../controls/build") # path dependent

FOLLOWER_SETPOINT = "FOLLOWER_SETPOINT"
DRONE_STATUS = "DRONE_STATUS"

class GroundNetwork(Node):

    def __init__(self):
        super().__init__('ground_network')

    def subscribe_drone_status(self):
        self.subscription = self.create_subscription(
            DroneStatus,
            DRONE_STATUS,
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.drone_id)

def main(args=None):
    rclpy.init(args=args)

    ground_network = GroundNetwork()
    rclpy.spin(ground_network)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
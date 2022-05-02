import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.load_gt = self.create_subscription(Odometry,
                                                'slung/load',
                                                self.load_gt_callback, qos_profile_sensor_data)


    def load_gt_callback(self, msg):
        time_s = msg.header.stamp.sec
        time_ns = msg.header.stamp.nanosec
        print(time)
        #x = msg.pose[-1].position.x
        #y = msg.pose[-1].position.x
        #z = msg.pose[-1].position.z
        #vx = msg.twist[-1].linear.x
        #vy = msg.twist[-1].linear.y
        #vz = msg.twist[-1].linear.z
        #print("x: %4.2f \t y: %4.2f \t z: %4.2f" %(x, y, z))
       # print("vx: %4.2f \t vy: %4.2f \t vz: %4.2f \n" %(vx, vy, vz))


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
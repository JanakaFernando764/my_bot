import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.05)   # meters
        self.declare_parameter('wheel_base', 0.35)     # front-back distance (L)
        self.declare_parameter('track_width', 0.30)    # left-right distance (W)

        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value
        self.W = self.get_parameter('track_width').value

        # Subscribers & Publishers
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        L, W, R = self.L, self.W, self.R

        # Mecanum kinematics
        v_fl = (1/R) * (vx - vy - (L+W)*wz)
        v_fr = (1/R) * (vx + vy + (L+W)*wz)
        v_rl = (1/R) * (vx + vy - (L+W)*wz)
        v_rr = (1/R) * (vx - vy + (L+W)*wz)

        msg_out = Float64MultiArray()
        msg_out.data = [v_fl, v_fr, v_rl, v_rr]
        self.pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

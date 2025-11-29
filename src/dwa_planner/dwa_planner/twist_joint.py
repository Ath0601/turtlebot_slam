# twist_to_joint_trajectory.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray

class TwistToEffort(Node):
    def __init__(self):
        super().__init__('twist_to_effort')
        
        # Robot parameters
        self.wheel_separation = 0.287  # meters
        self.wheel_radius = 0.033     # meters
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to joint trajectory controller
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )
        
        self.get_logger().info('Twist to Effort converter started')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Convert to wheel angular velocities
        left_w = (2*v - w*self.wheel_separation) / (2*self.wheel_radius)
        right_w = (2*v + w*self.wheel_separation) / (2*self.wheel_radius)

        # Simple proportional mapping to effort
        K = 1.0
        left_eff = K * left_w
        right_eff = K * right_w

        effort_msg = Float64MultiArray()
        effort_msg.data = [left_eff, right_eff]
        left_eff = max(min(left_eff, 1.5), -1.5)
        right_eff = max(min(right_eff, 1.5), -1.5)


        self.effort_pub.publish(effort_msg)
        self.get_logger().info(f"Effort: L={left_eff:.2f}, R={right_eff:.2f}")

def main():
    rclpy.init()
    node = TwistToEffort()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
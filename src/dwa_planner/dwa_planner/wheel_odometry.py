# wheel_odometry.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Robot parameters
        self.wheel_radius = 0.033
        self.wheel_separation = 0.287
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_time = self.get_clock().now()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.get_logger().info('Wheel Odometry node started')

    def joint_callback(self, msg):
        try:
            # Find wheel indices
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
            
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt <= 0:
                return
            
            # Calculate wheel displacements
            left_delta = msg.position[left_idx] - self.last_left_pos
            right_delta = msg.position[right_idx] - self.last_right_pos
            
            # Calculate linear and angular displacements
            linear_delta = (self.wheel_radius / 2.0) * (right_delta + left_delta)
            angular_delta = (self.wheel_radius / self.wheel_separation) * (right_delta - left_delta)
            
            # Update pose
            self.x += linear_delta * math.cos(self.theta)
            self.y += linear_delta * math.sin(self.theta)
            self.theta += angular_delta
            
            # Normalize theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Publish odometry
            self.publish_odometry(current_time.to_msg())
            
            # Update last values
            self.last_left_pos = msg.position[left_idx]
            self.last_right_pos = msg.position[right_idx]
            self.last_time = current_time
            
        except ValueError as e:
            self.get_logger().warn(f'Waiting for wheel joints: {e}')

    def publish_odometry(self, timestamp):
        # Publish TF
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        
        # Add covariance
        odom.pose.covariance[0] = 0.1  # x
        odom.pose.covariance[7] = 0.1  # y
        odom.pose.covariance[35] = 0.2  # yaw
        
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
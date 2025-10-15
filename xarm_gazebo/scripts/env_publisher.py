#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class EnvPosePublisher(Node):
    def __init__(self):
        super().__init__('env_pose_publisher')
        
        # Create publishers
        self.block_publisher = self.create_publisher(TFMessage, '/model/block/pose', 10)
        self.goal_publisher = self.create_publisher(TFMessage, '/model/goal/pose', 10)
        self.table_publisher = self.create_publisher(TFMessage, '/model/table_box/pose', 10)
        
        # Create timer, publish once per second
        self.timer = self.create_timer(1.0, self.publish_poses)
        
        self.get_logger().info('Environment Pose Publisher started')
    
    def publish_poses(self):
        """Publish environment pose data"""
        current_time = self.get_clock().now()
        
        # Publish block pose
        block_msg = self.create_tf_message('block', [0.4, 0.0, 0.142], [0.0, 0.0, 0.0], current_time)
        self.block_publisher.publish(block_msg)
        self.get_logger().info('Published block pose')
        
        # Publish goal pose
        goal_msg = self.create_tf_message('goal', [0.7, 0.2, 0.108], [0.0, 0.0, 0.524], current_time)
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info('Published goal pose')
        
        # Publish table pose
        table_msg = self.create_tf_message('table_box', [0.5, 0.0, 0.054], [0.0, 0.0, 0.0], current_time)
        self.table_publisher.publish(table_msg)
        self.get_logger().info('Published table pose')
    
    def create_tf_message(self, model_name, position, orientation_euler, timestamp):
        """Create TF message"""
        import math
        
        tf_msg = TransformStamped()
        tf_msg.header = Header()
        tf_msg.header.stamp = timestamp.to_msg()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = model_name
        
        tf_msg.transform.translation.x = position[0]
        tf_msg.transform.translation.y = position[1]
        tf_msg.transform.translation.z = position[2]
        
        # Convert Euler angles to quaternion
        roll, pitch, yaw = orientation_euler
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        tf_msg.transform.rotation.w = cr * cp * cy + sr * sp * sy
        tf_msg.transform.rotation.x = sr * cp * cy - cr * sp * sy
        tf_msg.transform.rotation.y = cr * sp * cy + sr * cp * sy
        tf_msg.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        tf_message = TFMessage()
        tf_message.transforms = [tf_msg]
        
        return tf_message

def main(args=None):
    rclpy.init(args=args)
    node = EnvPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

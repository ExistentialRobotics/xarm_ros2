#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class EnvPosePublisher(Node):
    def __init__(self):
        super().__init__('env_pose_publisher')
        
        # Create publishers for static poses only (goal and table)
        # Block pose is handled by parameter_bridge for dynamic updates
        self.goal_publisher = self.create_publisher(TFMessage, '/model/goal/pose', 10)
        self.table_publisher = self.create_publisher(TFMessage, '/model/table_box/pose', 10)
        
        # Also publish to /tf for TF tree compatibility
        self.tf_publisher = self.create_publisher(TFMessage, '/tf', 10)
        
        # Create timer, publish once per second
        self.timer = self.create_timer(1.0, self.publish_static_poses)
        
        # Flag to track if poses have been published successfully at least once
        self.published_success = False
        
        self.get_logger().info('Static Environment Pose Publisher started!')
    
    def publish_static_poses(self):
        """Publish static environment pose data"""
        try:
            current_time = self.get_clock().now()
            
            # NOTE: Block pose is now handled by parameter_bridge for real-time dynamic updates
            # Only publish static reference poses for goal and table

            # Candidate goal positions
            goal_pos_list = [
                [0.65,  0.0, 0.5],   # center
                [ 0.7,  0.2, 0.5],   # left
                [ 0.7, -0.2, 0.5],   # right
                [0.65,  0.2, 0.5],   # left_lower
            ]

            # Candidate goal orientation list (Euler angles)
            goal_ori_list = [
                [0.0, 0.0,    0.0],   #   0 degree
                [0.0, 0.0,  0.523],   #  30 degree
                [0.0, 0.0, -0.523],   # -30 degree
                [0.0, 0.0,  0.785],   #  45 degree
                [0.0, 0.0, -0.785],   # -45 degree
                [0.0, 0.0,  1.047],   #  60 degree
                [0.0, 0.0, -1.047],   # -60 degree
                [0.0, 0.0,  1.570],   #  90 degree
            ]

            # Test goal pose
            goal_pose_c_00 = goal_pos_list[0] + goal_ori_list[0] # Rotation   0 deg
            goal_pose_l_30 = goal_pos_list[1] + goal_ori_list[1] # Rotation  30 deg
            goal_pose_r_30 = goal_pos_list[2] + goal_ori_list[2] # Rotation -30 deg
            goal_pose_l_45 = goal_pos_list[1] + goal_ori_list[3] # Rotation  45 deg
            goal_pose_r_45 = goal_pos_list[2] + goal_ori_list[4] # Rotation -45 deg
            goal_pose_l_60 = goal_pos_list[3] + goal_ori_list[5] # Rotation  60 deg
            goal_pose_r_60 = goal_pos_list[3] + goal_ori_list[6] # Rotation -60 deg
            goal_pose_l_90 = goal_pos_list[3] + goal_ori_list[7] # Rotation  90 deg

            # Create goal and table messages
            goal_pose = goal_pose_l_30
            goal_msg = self.create_tf_message('goal', goal_pose[:3], goal_pose[3:], current_time)
            table_msg = self.create_tf_message('table_box', [0.5, 0.0, 0.054], [0.0, 0.0, 0.0], current_time)
            
            # Publish to individual model pose topics
            self.goal_publisher.publish(goal_msg)
            self.table_publisher.publish(table_msg)
            
            # Also publish to /tf for TF tree compatibility
            # Combine messages for single TF publication
            combined_tf_msg = TFMessage()
            combined_tf_msg.transforms = goal_msg.transforms + table_msg.transforms
            self.tf_publisher.publish(combined_tf_msg)
            
            # Log success message only once
            if not self.published_success:
                # self.get_logger().info('Static environment poses published successfully!')
                # self.get_logger().info('Publishing goal pose: [0.7, 0.2, 0.108] with rotation [0.0, 0.0, 0.524]')
                # self.get_logger().info('Publishing table pose: [0.5, 0.0, 0.054] with rotation [0.0, 0.0, 0.0]')
                self.published_success = True
            else:
                # Log periodically for debugging
                if hasattr(self, '_debug_counter'):
                    self._debug_counter += 1
                else:
                    self._debug_counter = 1
                
                # if self._debug_counter % 10 == 0:  # Log every 10 seconds
                #     self.get_logger().info(f'Published {self._debug_counter} times. Goal and table poses active.')
                    
        except Exception as e:
            self.get_logger().error(f'Error publishing poses: {e}')
    
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
        node.get_logger().info('Environment states publisher halted!')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

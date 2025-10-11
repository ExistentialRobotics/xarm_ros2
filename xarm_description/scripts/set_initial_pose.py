#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Create publisher
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/xarm6_traj_controller/joint_trajectory', 
            10
        )
        
        # Wait for system to start
        self.get_logger().info('Waiting for system to start...')
        time.sleep(3.0)  # Wait 3 seconds to ensure controller is loaded
        
        # Set initial pose
        self.set_initial_pose()
        
    def set_initial_pose(self):
        """Set initial pose of the robot arm"""
        msg = JointTrajectory()
        
        # Set joint names
        msg.joint_names = [
            'xarm6_joint1',
            'xarm6_joint2', 
            'xarm6_joint3',
            'xarm6_joint4',
            'xarm6_joint5',
            'xarm6_joint6'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        
        # Set target position
        point.positions = [0.0, -0.78, -0.78, 0.0, 1.5708, 0.0]
        
        # Set time (5 seconds to reach target position)
        point.time_from_start = Duration(sec=5, nanosec=0)
        
        # Add trajectory point
        msg.points = [point]
        
        # Publish message
        self.get_logger().info('Set initial pose of the robot arm: [0.0, -0.78, -0.78, 0.0, 1.5708, 0.0]')
        self.publisher.publish(msg)
        
        # Wait for execution to complete
        # time.sleep(6.0)
        self.get_logger().info('Initial pose set complete')

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = InitialPoseSetter()
        # Don't spin, just wait for the task to complete
        # The node will handle its own shutdown
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors

if __name__ == '__main__':
    main()

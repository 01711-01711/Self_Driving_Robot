'''
To control the robot's movements and actions
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/plan', self.path_callback, 10)
        
        # Placeholder for the current path
        self.current_path = None
        self.current_path_index = 0


    def path_callback(self, msg):
        # Callback function to handle incoming path data
        self.current_path = msg.poses
        self.current_path_index = 0
        self.get_logger().info('New path received')
        self.follow_path()


    def follow_path(self):
        # Function to start following the current path
        if self.current_path is None:
            self.get_logger().info('No path to follow')
            return

        for pose in self.current_path:
            self.move_to_pose(pose)
            self.current_path_index += 1
            # Check if we should preempt path following for some reason
            if self.should_preempt_path_following():
                break


    def move_to_pose(self, pose):
        # Function to move the robot to a specific pose
        # Here include logic to calculate the required velocity to reach the pose
        twist_msg = Twist()
        # ... set twist_msg values to move towards the pose ...
        self.velocity_publisher.publish(twist_msg)
        # Here include logic to determine when the robot has reached the pose


    def should_preempt_path_following(self):
        # Function to check if we should stop following the path
        # For example, check for a cancel message or if the robot is close enough to the goal
        return False



def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

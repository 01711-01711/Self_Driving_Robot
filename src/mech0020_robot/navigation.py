'''
Contains algorithms for pathfinding and obstacle avoidance
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist


class HospitalRobotNavigator(Node):
    def __init__(self):
        super().__init__('hospital_robot_navigator')
        
        # Initialization of the subscriber to the map topic and robot's pose
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
            
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10)

        # Initialization of the publisher to the cmd_vel topic
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Placeholder for the map data and robot's pose
        self.map_data = None
        self.robot_pose = None


    def map_callback(self, msg):
        # Callback function to handle incoming map data
        self.map_data = msg.data
        self.get_logger().info('Map data received')


    def pose_callback(self, msg):
        # Callback function to handle incoming robot pose data
        self.robot_pose = msg.pose
        self.get_logger().info('Robot pose received')


    def navigate_to_goal(self, goal_pose):
        # Main navigation function to move the robot to the goal
        if self.map_data is None or self.robot_pose is None:
            self.get_logger().info('Waiting for map and pose data...')
            return
        
        # Here you would implement your A* pathfinding algorithm
        # For now, we just log that we would navigate
        self.get_logger().info('Navigating to goal...')
        
        # Calculate the path to the goal using A*
        path = self.calculate_path(self.robot_pose, goal_pose)
        
        # Follow the calculated path
        for waypoint in path:
            self.move_to_waypoint(waypoint)


    def calculate_path(self, start_pose, goal_pose):
        # Stub for path calculation method
        # A* algorithm implementation
        path = []
        # ... calculate the path ...
        return path


    def move_to_waypoint(self, waypoint):
        # Stub for waypoint movement method
        # Send Twist messages to move the robot
        twist_msg = Twist()
        # ... set twist_msg values to move towards the waypoint ...
        self.velocity_publisher.publish(twist_msg)



def main(args=None):
    rclpy.init(args=args)
    navigator = HospitalRobotNavigator()
    
    # Define goal pose here. For example:
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 10
    goal_pose.pose.position.y = 5
    goal_pose.pose.orientation.w = 1
    
    # Start navigation
    navigator.navigate_to_goal(goal_pose)
    
    # Spin the ROS node so that it processes data and keeps running
    rclpy.spin(navigator)

    # Destroy the node explicitly
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
The main script to run application
'''

import rclpy
from navigation import HospitalRobotNavigator
from robot_controller import RobotController
from sensor_handler import SensorHandler


def main(args=None):
    rclpy.init(args=args)
    
    # Create instances of your custom nodes
    navigator = HospitalRobotNavigator()
    controller = RobotController()
    sensor_handler = SensorHandler()

    # Use threads or AsyncIO to spin each node in its own context
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(navigator)
    executor.add_node(controller)
    executor.add_node(sensor_handler)

    try:
        print("Spinning nodes...")
        # Spin the nodes in separate threads
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown and cleanup
        executor.shutdown()
        navigator.destroy_node()
        controller.destroy_node()
        sensor_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

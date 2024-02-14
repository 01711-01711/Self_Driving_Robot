'''
Utility functions used across the project
'''

import math
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion


def euclidean_distance(point1, point2):
    """
    Calculate the Euclidean distance between two points
    """
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)


def quaternion_to_euler(quaternion):
    """
    Convert a quaternion to Euler angles
    """
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = euler_from_quaternion(q)
    return euler  # roll, pitch, yaw


def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    """
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z  # in radians


def create_pose(x, y, z, roll, pitch, yaw):
    """
    Create a Pose message from position and orientation
    """
    pose = Pose()
    pose.position = Point(x, y, z)
    quat = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation = Quaternion(*quat)
    return pose


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]


def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle



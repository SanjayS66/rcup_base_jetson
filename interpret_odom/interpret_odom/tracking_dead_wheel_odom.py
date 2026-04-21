#!/usr/bin/env python3

"""
tracking_dead_wheel_node.py -- IMU-Aided dead-wheel odometry.
Updated to handle JSON format with timestamp 't'
"""


import numpy as np

if not hasattr(np, "float"):
    np.float = float
if not hasattr(np, "maximum_sctype"):
    np.maximum_sctype = lambda t: np.float64

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, PoseStamped
from std_msgs.msg import Header, String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
import tf_transformations
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import json
import math
from collections import deque


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class DeadWheelOdomNode(Node):
    def __init__(self):
        super().__init__('dead_wheel_odom')
        
        self.declare_parameter('ticks_per_rev_front', 2400)
        self.declare_parameter('ticks_per_rev_left', 2400)
        self.declare_parameter('ticks_per_rev_right', 2400)
        self.declare_parameter('wheel_radius',0.02938)#0.03
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('pods', [
            0.0685, -0.007, math.pi/2.0,
            -0.04, 0.07, 0.0,
            -0.045, -0.074, 0.0
        ])
        self.declare_parameter('inputs_front_left_right', ['a', 'c', 'b'])
        self.declare_parameter('encoder_signs', [1, -1, 1])
        self.declare_parameter('frame_imu', 'imu_link')
        self.declare_parameter('imu_offset_x', -0.095)
        self.declare_parameter('imu_offset_y', 0.13)
        self.declare_parameter('imu_offset_z', 0.0)

        self.tpr_front = float(self.get_parameter('ticks_per_rev_front').value)
        self.tpr_left = float(self.get_parameter('ticks_per_rev_left').value)
        self.tpr_right = float(self.get_parameter('ticks_per_rev_right').value)
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value
        self.input_map = self.get_parameter('inputs_front_left_right').value
        self.encoder_signs = self.get_parameter('encoder_signs').value
        
        pods_param = self.get_parameter('pods').value
        self.pods = [tuple(pods_param[i:i+3]) for i in range(0, len(pods_param), 3)]

        self.prev_ticks = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_time = None

        self.lateral_idx = 0
        self.left_idx = 1
        self.right_idx = 2

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.path_pub = self.create_publisher(Path, 'odom_path', 10)
        self.path_history = deque(maxlen=1000)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        imu_tf = TransformStamped()
        imu_tf.header.stamp = self.get_clock().now().to_msg()
        imu_tf.header.frame_id = self.frame_base
        imu_tf.child_frame_id = self.get_parameter('frame_imu').value
        imu_tf.transform.translation.x = self.get_parameter('imu_offset_x').value
        imu_tf.transform.translation.y = self.get_parameter('imu_offset_y').value
        imu_tf.transform.translation.z = self.get_parameter('imu_offset_z').value
        imu_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(imu_tf)

        self.create_subscription(String, 'mcu/imu', self._imu_cb, 25)
        self.create_subscription(String, 'mcu/enc', self._enc_cb, 25)
        self.yaw_offset = None        
        self.yaw_offset = None  
        

    def _handle_sample(self, d_front_raw, d_left_raw, d_right_raw, imu_yaw_rad, ros_now):
        ticks = np.array([d_front_raw, d_left_raw, d_right_raw], dtype=np.int64)
        current_time_sec = ros_now.sec + ros_now.nanosec * 1e-9
        
        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.prev_time = current_time_sec
            return True

        # Calculate dt using the global ROS clock
        dt = current_time_sec - self.prev_time
        if dt <= 0:
            dt = 1e-3

        d_ticks = ticks - self.prev_ticks
        circ = 2.0 * math.pi * self.wheel_r
        
        d_front = d_ticks[0] * (circ / self.tpr_front)
        d_left = d_ticks[1] * (circ / self.tpr_left)
        d_right = d_ticks[2] * (circ / self.tpr_right)

        y_left = self.pods[self.left_idx][1]
        y_right = self.pods[self.right_idx][1]
        track_width = abs(y_left - y_right) if abs(y_left - y_right) > 1e-4 else 1.0

        body_dx = (d_left * abs(y_right) + d_right * abs(y_left)) / track_width
        imu_dtheta = math.atan2(math.sin(imu_yaw_rad - self.yaw), math.cos(imu_yaw_rad - self.yaw))
        
        front_lever_arm_x = self.pods[self.lateral_idx][0]
        body_dy = d_front - (imu_dtheta * front_lever_arm_x)

        # midpoint_yaw = self.yaw + (imu_dtheta / 2.0)
        # cos_h, sin_h = math.cos(midpoint_yaw), math.sin(midpoint_yaw)
        cos_h, sin_h = math.cos(imu_yaw_rad), math.sin(imu_yaw_rad)
        
        step_dx_world = body_dx * cos_h - body_dy * sin_h
        step_dy_world = body_dx * sin_h + body_dy * cos_h

        self.x += step_dx_world
        self.y += step_dy_world

        odom = Odometry()
        odom.header.stamp = ros_now
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        odom.twist.twist.linear.x = body_dx / dt
        odom.twist.twist.linear.y = body_dy / dt
        odom.twist.twist.angular.z = imu_dtheta / dt

        odom.pose.covariance = [1e-9] * 36
        odom.pose.covariance[0] = 1e-5   # X translation (high confidence)
        odom.pose.covariance[7] = 1e-5   # Y translation (high confidence)
        odom.pose.covariance[35] = 1e-3  
        odom.twist.covariance = [1e-9] * 36
        odom.twist.covariance[0] = 1e-3   # vx
        odom.twist.covariance[7] = 1e-3   # vy
        odom.twist.covariance[35] = 1e-3  # vyaw

        self.odom_pub.publish(odom)

        t_odom = TransformStamped()
        t_odom.header.stamp = ros_now
        t_odom.header.frame_id = self.frame_odom
        t_odom.child_frame_id = self.frame_base
        t_odom.transform.translation.x = self.x
        t_odom.transform.translation.y = self.y
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation = odom.pose.pose.orientation
        # self.tf_broadcaster.sendTransform(t_odom)

        self.get_logger().info(
            f"Ticks : ({ticks[0]}, {ticks[1]}, {ticks[2]}) | "
            f"Body Δ(x,y): ({body_dx:.4f}, {body_dy:.4f}) m | "
            f"World Δ(x,y): ({step_dx_world:.4f}, {step_dy_world:.4f}) m | "
            f"Pose(x,y,yaw): ({self.x:.4f} m, {self.y:.4f} m, {math.degrees(self.yaw):.2f}°)"
        )

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        self.path_history.append(ps)
        
        path_msg = Path()
        path_msg.header = odom.header
        path_msg.poses = list(self.path_history)
        self.path_pub.publish(path_msg)

        self.prev_ticks = ticks
        self.prev_time = current_time_sec
        return True

    def _imu_cb(self, msg: String):
        try:
            data = json.loads(msg.data.strip())
        except json.JSONDecodeError:
            return

        ros_now = self.get_clock().now().to_msg()
        
        qx, qy, qz, qw = float(data.get("qx", 0.0)), float(data.get("qy", 0.0)), float(data.get("qz", 0.0)), float(data.get("qw", 1.0))
        _, _, raw_yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

        if self.yaw_offset is None:
            self.yaw_offset = raw_yaw
            
        self.yaw = raw_yaw - self.yaw_offset

        try:
            imu_msg = Imu()
            imu_msg.header.stamp = ros_now
            imu_msg.header.frame_id = self.get_parameter('frame_imu').value
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = qx, qy, qz, qw
            
            imu_msg.angular_velocity.x = math.radians(float(data.get("gx", 0.0)))
            imu_msg.angular_velocity.y = math.radians(float(data.get("gy", 0.0)))
            imu_msg.angular_velocity.z = math.radians(float(data.get("gz", 0.0)))

            imu_msg.linear_acceleration.x = float(data.get("ax", 0.0))
            imu_msg.linear_acceleration.y = float(data.get("ay", 0.0))
            imu_msg.linear_acceleration.z = float(data.get("az", 0.0))

            imu_msg.orientation_covariance = [1e-9] * 9
            imu_msg.orientation_covariance[8] = 1e-5  
            imu_msg.angular_velocity_covariance = [1e-9] * 9
            imu_msg.angular_velocity_covariance[8] = 1e-5 

            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"IMU pub error: {e}")

    def _enc_cb(self, msg: String):
        try:
            data = json.loads(msg.data.strip())
        except json.JSONDecodeError:
            return

        ros_now = self.get_clock().now().to_msg()

        # If IMU hasn't initialized self.yaw yet, skip encoder update
        if not hasattr(self, 'yaw') or self.yaw_offset is None:
            return

        try:
            vals = {
                'a': int(data.get('a', 0)) * self.encoder_signs[0],
                'b': int(data.get('b', 0)) * self.encoder_signs[1],
                'c': int(data.get('c', 0)) * self.encoder_signs[2]
            }
            f = vals.get(self.input_map[0], 0)
            l = vals.get(self.input_map[1], 0)
            r = vals.get(self.input_map[2], 0)

            self._handle_sample(f, l, r, self.yaw, ros_now)
        except Exception as e:
            self.get_logger().warn(f"Mapping error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DeadWheelOdomNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

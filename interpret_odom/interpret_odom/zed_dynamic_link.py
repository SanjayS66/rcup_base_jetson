import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class DynamicZedTF(Node):
    def __init__(self):
        super().__init__('dynamic_zed_tf')
        self.br = TransformBroadcaster(self)
        
        # Subscribe to ZED IMU topic. Verify this matches your active topic.
        self.subscription = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data', 
            self.imu_callback,
            10)
            
    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'zed_camera_link'
        
        # 1. Static Pivot Coordinates (Set these to the exact pivot hinge location)
        pivot_x = 0.2765
        pivot_y = 0.1350
        pivot_z = 0.6000 
        
        # 2. Camera Offset from Pivot (Distance from hinge to camera center)
        off_x = 0.0
        off_y = 0.0
        off_z = 0.0370 
        
        # 3. Extract pure Pitch relative to gravity (Ignores World Yaw/Roll drift)
        q = msg.orientation
        
        # Project world Z-axis (gravity) into local camera frame
        gx = 2.0 * (q.x * q.z - q.w * q.y)
        gz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        
        # Calculate isolated pitch
        pitch = math.atan2(-gx, gz)
        
        # 4. Rotate the offset vector (Simplified matrix for Pitch only)
        rot_x = math.cos(pitch) * off_x + math.sin(pitch) * off_z
        rot_y = off_y # No change in Y during pitch
        rot_z = -math.sin(pitch) * off_x + math.cos(pitch) * off_z
        
        # 5. Apply final dynamic translation
        t.transform.translation.x = pivot_x + rot_x
        t.transform.translation.y = pivot_y + rot_y
        t.transform.translation.z = pivot_z + rot_z

        # 6. Apply quaternion rotation (Pure pitch)
        t.transform.rotation.w = math.cos(pitch * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(pitch * 0.5)
        t.transform.rotation.z = 0.0

        self.br.sendTransform(t)
def main():
    rclpy.init()
    node = DynamicZedTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
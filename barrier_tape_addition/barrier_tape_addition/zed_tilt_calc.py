#ZED CAM TIL CALCULATOR TO GET THE PITCH TO GIVE IN THE STATIC TRANSFORM FROM BASE_LINK TO ZED_CAM

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math



class ZEDImuSubscriber(Node):
    def __init__(self):
        super().__init__('zed_imu_subscriber')
        
        # Subscribe to the ZED IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_callback,
            10 # QoS profile depth
        )
        self.get_logger().info("Subscribed to /zed/zed_node/imu/data. Waiting for messages...")

    def imu_callback(self, msg):
        # 1. Extract the quaternion values
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w
        
        # 2. Convert quaternion to Euler angles (Roll, Pitch, Yaw)
        roll, pitch, yaw = self.euler_from_quaternion(q_x, q_y, q_z, q_w)

        # 3. Convert radians to degrees for easier human reading
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # 4. Print the results to the terminal

        self.get_logger().info(
            f'\n--- Current Orientation ---\n'
            f'Quaternion : x={q_x:.4f}, y={q_y:.4f}, z={q_z:.4f}, w={q_w:.4f}\n'
            f'Euler (Rad): Roll={roll:.4f}, Pitch={pitch:.4f}, Yaw={yaw:.4f}\n'
            f'Euler (Deg): Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}°\n'
            f'---------------------------'
        )


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        # Clamp t2 to prevent NaN errors due to floating point inaccuracies
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    
    imu_subscriber = ZEDImuSubscriber()
    
    try:
        # Spin the node so the callback function is called
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
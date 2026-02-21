import yaml
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ShelfBroadcaster(Node):
    def __init__(self):
        super().__init__('publish_shelves')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_transforms("/home/rm/base_2025_ws/src/map_transforms_creator/transforms/shelves.yaml")

    def publish_transforms(self, filename):
        try:
            with open(filename, 'r') as f:
                shelves = yaml.safe_load(f)
                
            transforms = []
            for name, data in shelves.items():
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'map'
                t.child_frame_id = name
                
                t.transform.translation.x = data['x']
                t.transform.translation.y = data['y']
                t.transform.translation.z = data['z']
                t.transform.rotation.x = data['qx']
                t.transform.rotation.y = data['qy']
                t.transform.rotation.z = data['qz']
                t.transform.rotation.w = data['qw']
                
                transforms.append(t)
            
            self.broadcaster.sendTransform(transforms)
            self.get_logger().info(f"Published {len(transforms)} shelves.")

        except FileNotFoundError:
            self.get_logger().error("shelves.yaml not found!")

def main():
    rclpy.init()
    node = ShelfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
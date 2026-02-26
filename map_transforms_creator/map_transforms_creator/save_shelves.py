import sys
import os
import yaml
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class ShelfSaver(Node):
    def __init__(self):
        super().__init__('save_shelves')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def save_pose(self, shelf_name, filename="/home/rm/base_2025_ws/src/map_transforms_creator/transforms/shelves.yaml"):
        try:
            # Wait for the transform from map to base_link
            t = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            data = {
                shelf_name: {
                    'x': t.transform.translation.x,
                    'y': t.transform.translation.y,
                    'z': t.transform.translation.z,
                    'qx': t.transform.rotation.x,
                    'qy': t.transform.rotation.y,
                    'qz': t.transform.rotation.z,
                    'qw': t.transform.rotation.w
                }
            }

            # Append to YAML file
            mode = 'a' if os.path.exists(filename) else 'w'
            with open(filename, mode) as f:
                yaml.dump(data, f)
                
            self.get_logger().info(f"Saved {shelf_name} to {filename}")

        except Exception as e:
            self.get_logger().error(f"Could not save pose: {e}")

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print("Usage: save_shelf <shelf_name>")
        return

    node = ShelfSaver()
    shelf_name = sys.argv[1]

    # 1. Start spinning in the background so the TF buffer can fill
    # We use a loop to give it a few tries
    max_tries = 50
    found = False
    
    node.get_logger().info(f"Waiting for transform 'map' -> 'base_link'...")
    
    for i in range(max_tries):
        rclpy.spin_once(node, timeout_sec=0.5) # Process incoming TF messages
        
        # 2. Check if the transform is now available
        if node.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)):
            node.save_pose(shelf_name)
            found = True
            break
        else:
            node.get_logger().warn(f"Still waiting... (Attempt {i+1}/{max_tries})")

    if not found:
        node.get_logger().error("Could not find transform after waiting. Is localization running?")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
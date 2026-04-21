#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.duration import Duration

# Corrected Import
from tape_msgs.msg import BoundingBoxes

class TapeMapModifier(Node):
    def __init__(self):
        super().__init__('tape_map_modifier')

        self.declare_parameter('boxes_topic', '/tape_bounding_boxes')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/map_with_tapes')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_timeout_sec', 0.25)

        self.boxes_topic = self.get_parameter('boxes_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_msg = None
        self.map_lock = False
        self.tape_cells = set()

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        boxes_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.create_subscription(BoundingBoxes, self.boxes_topic, self._boxes_cb, boxes_qos)        
        
        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, map_qos)
        self.create_service(Empty, 'clear_tape_obstacles', self._clear_srv)

        self.create_timer(1.0, self._publish_modified_map)
        self.get_logger().info(f"TapeMapModifier initialized. Listening to {self.boxes_topic}.")

    def _clear_srv(self, request, response):
        self.tape_cells.clear()
        self.get_logger().info("Tape obstacles cleared.")
        return response

    def _boxes_cb(self, msg: BoundingBoxes):
        self.get_logger().info(f"[DEBUG 1] Received {len(msg.boxes)} boxes.")
        
        if self.map_msg is None:
            self.get_logger().warn("[DEBUG 1A] Map not yet received. Skipping boxes.", throttle_duration_sec=5.0)
            return

        source_frame = msg.header.frame_id if msg.header.frame_id else msg.boxes[0].header.frame_id

        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
        except Exception as e:
            self.get_logger().warn(f"[DEBUG 2] TF lookup failed: {e}", throttle_duration_sec=2.0)
            return

        for box in msg.boxes:
            pts_cam = [
                (box.corner1.x, box.corner1.y, box.corner1.z),
                (box.corner2.x, box.corner2.y, box.corner2.z),
                (box.corner3.x, box.corner3.y, box.corner3.z),
                (box.corner4.x, box.corner4.y, box.corner4.z),
            ]

            map_pts = []
            for (x, y, z) in pts_cam:
                p = PointStamped()
                p.header.frame_id = source_frame
                p.header.stamp = msg.header.stamp
                p.point.x = float(x)
                p.point.y = float(y)
                p.point.z = float(z)

                try:
                    p_map = do_transform_point(p, t)
                    map_pts.append((p_map.point.x, p_map.point.y))
                except Exception as e:
                    self.get_logger().error(f"Point transform error: {e}")

            if len(map_pts) == 4:
                p_center = PointStamped()
                p_center.header.frame_id = source_frame
                p_center.point.x = float(box.center.x)
                p_center.point.y = float(box.center.y)
                p_center.point.z = float(box.center.z)
                p_center_map = do_transform_point(p_center, t)
                
                self.get_logger().info(f"[DEBUG 3] Tape center in {self.map_frame}: ({p_center_map.point.x:.3f}, {p_center_map.point.y:.3f})")
                self._mark_tape_polygon(map_pts)

    def _map_cb(self, msg: OccupancyGrid):
        if self.map_lock:
            return
        if self.map_msg is None:
            self.get_logger().info("Initial map received.")
        self.map_msg = msg

    def _mark_tape_polygon(self, poly_xy):
        info = self.map_msg.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        width = info.width
        height = info.height

        xs = [float(p[0]) for p in poly_xy]
        ys = [float(p[1]) for p in poly_xy]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        i_min = int(math.floor((min_x - origin_x) / res))
        i_max = int(math.ceil((max_x - origin_x) / res))
        j_min = int(math.floor((min_y - origin_y) / res))
        j_max = int(math.ceil((max_y - origin_y) / res))

        if i_max < 0 or j_max < 0 or i_min >= width or j_min >= height:
            self.get_logger().warn(f"[DEBUG 4] Polygon at ({min_x:.2f}, {min_y:.2f}) is outside map bounds.")
            return

        i_min = max(0, i_min)
        i_max = min(width - 1, i_max)
        j_min = max(0, j_min)
        j_max = min(height - 1, j_max)

        i_vals = np.arange(i_min, i_max + 1)
        j_vals = np.arange(j_min, j_max + 1)
        
        xs_centers = origin_x + (i_vals + 0.5) * res
        ys_centers = origin_y + (j_vals + 0.5) * res

        XX, YY = np.meshgrid(xs_centers, ys_centers)
        pts_x = XX.ravel()
        pts_y = YY.ravel()

        poly = np.array(poly_xy, dtype=float)
        px = poly[:, 0]
        py = poly[:, 1]
        n = len(px)
        inside = np.zeros(pts_x.shape, dtype=bool)

        for k in range(n):
            xi, yi = px[k], py[k]
            xj, yj = px[(k + 1) % n], py[(k + 1) % n]
            cond = ((yi > pts_y) != (yj > pts_y)) & (
                pts_x < (xj - xi) * (pts_y - yi) / (yj - yi + 1e-12) + xi
            )
            inside ^= cond

        inside_idx = np.nonzero(inside)[0]
        Ni = i_vals.size
        rows = inside_idx // Ni
        cols = inside_idx % Ni
        
        cells_added = 0
        for r, c in zip(rows, cols):
            j = int(j_vals[r])
            i = int(i_vals[c])
            self.tape_cells.add((i, j))
            cells_added += 1
                
        self.get_logger().info(f"[DEBUG 5] Added {cells_added} cells to map overlay.")

    def _publish_modified_map(self):
        if self.map_msg is None:
            return

        self.map_lock = True
        modified_map = OccupancyGrid()
        modified_map.header = self.map_msg.header
        modified_map.info = self.map_msg.info
        
        data = np.array(self.map_msg.data, dtype=np.int8).reshape(
            (self.map_msg.info.height, self.map_msg.info.width)
        )

        for (i, j) in self.tape_cells:
            if 0 <= i < self.map_msg.info.width and 0 <= j < self.map_msg.info.height:
                data[j, i] = 100

        modified_map.data = data.flatten().tolist()
        modified_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(modified_map)
        self.map_lock = False

def main(args=None):
    rclpy.init(args=args)
    node = TapeMapModifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
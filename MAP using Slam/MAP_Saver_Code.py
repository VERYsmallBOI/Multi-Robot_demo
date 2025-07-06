import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
import cv2

class SaveMap(Node):
    def __init__(self):
        super().__init__('save_map')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.callback, 10)
        self.received = False

    def callback(self, msg):
        if self.received:
            return

        self.get_logger().info("Saving map...")

        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create PGM image
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 205      # Unknown
        img[data == 0] = 254       # Free
        img[data > 0] = 0          # Occupied

        cv2.imwrite('atlantis.pgm', img)

        map_yaml = {
            'image': 'atlantis.pgm',
            'resolution': msg.info.resolution,
            'origin': [
                msg.info.origin.position.x,
                msg.info.origin.position.y,
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }

        with open('atlantis.yaml', 'w') as f:
            yaml.dump(map_yaml, f)

        self.get_logger().info("Saved map to 'atlantis.pgm' and 'atlantis.yaml'")
        self.received = True
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SaveMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


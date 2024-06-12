import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header
import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_array = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None

    def map_callback(self, msg):
        map_list = list(msg.data)
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_array = np.array(map_list).reshape((self.map_height, self.map_width))

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "*end\n"
        }
        self.get_logger().info(f"ODOMX: {odom_data['x']}")
        self.get_logger().info(f"ODOMY: {odom_data['y']}")
        self.update_map(odom_data["x"], odom_data["y"])

    def transform_to_cell(self, x, y, resolution):
        cellx = int(x / resolution)
        celly = int(y / resolution)
        return cellx, celly 

    def update_map(self, x=0, y=0):  
        if self.map_array is None:
            self.get_logger().warn("Map not received yet")
            return
        
        cellx, celly = self.transform_to_cell(x, y, self.map_resolution)
        
        if 0 <= cellx < self.map_width and 0 <= celly < self.map_height:
            self.map_array[celly, cellx] = -1  # Update the map cell with a specific value
            self.get_logger().info(f"Updated map at cell ({cellx}, {celly}) to -1")
            self.publish_updated_map()
        else:
            self.get_logger().warn(f"Transformed cell ({cellx}, {celly}) is out of map bounds")

    def publish_updated_map(self):
        updated_map_msg = OccupancyGrid()
        updated_map_msg.header = Header()
        updated_map_msg.header.stamp = self.get_clock().now().to_msg()
        updated_map_msg.header.frame_id = "map"
        updated_map_msg.info.resolution = self.map_resolution
        updated_map_msg.info.width = self.map_width
        updated_map_msg.info.height = self.map_height
        updated_map_msg.info.origin.position.x = 0.0
        updated_map_msg.info.origin.position.y = 0.0
        updated_map_msg.info.origin.position.z = 0.0
        updated_map_msg.info.origin.orientation.x = 0.0
        updated_map_msg.info.origin.orientation.y = 0.0
        updated_map_msg.info.origin.orientation.z = 0.0
        updated_map_msg.info.origin.orientation.w = 1.0
        updated_map_msg.data = self.map_array.flatten().tolist()
        self.map_pub.publish(updated_map_msg)

def main(args=None):
    rclpy.init(args=args)
    localization = Localization()
    rclpy.spin(localization)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

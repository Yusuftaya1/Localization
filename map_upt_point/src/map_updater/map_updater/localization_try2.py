import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_data = None
        self.odom_data = None
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def map_callback(self, msg):
        map_list = list(msg.data)
        self.map_json = {
            "msg_type": "MAP",
            "width": msg.info.width,#112
            "height": msg.info.height,#103
            "resolution": msg.info.resolution,
            "data": map_list,
            "end": "*end\n"
        }
        self.map_array = np.array(map_list).reshape(int(self.map_json["width"]), int(self.map_json["height"]))
        print(self.map_array.shape)

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "*end\n"
        }

    def transform_to_cell(self,x,y,resolution):
        self.reso=resolution
        self.cellx = int(x/self.reso)
        self.celly = int(y/self.reso)
        #self.cellw = 
        return self.cellx, self.celly #self.cellw
    
    def update_map(self):
        cellx , celly = self.transform_to_cell(self.odom_data["x"],self.odom_data["y"],self.map_data["resolution"])#,odom_data["w"]

def main(args=None):
    rclpy.init(args=args)

    localization = Localization()

    rclpy.spin(localization)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
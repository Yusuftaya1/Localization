import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData , Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class MapSub(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

    def map_callback(self, msg):
        map_data = list(msg.data)

        map_json = {
            "msg_type": "MAP",
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "data": map_data,
            "end": "**********end***********\n"
        }
        self.localization = Localization(map_json=map_json)

class OdomSub(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "**********end***********\n"
        }
        self.localization = Localization(odom_data=odom_data)

class Localization():
    def __init__(self, map_json=None, odom_data=None):
        self.map_data = map_json
        self.odom_data = odom_data

    def print_map_data(self):
        print("Map Data:")
        print("Message Type:", self.map["msg_type"])
        print("Width:", self.map["width"])
        print("Height:", self.map["height"])
        print("Resolution:", self.map["resolution"])
        print("Data:", self.map["data"])
        print("End:", self.map["end"])

    def print_odom_data(self):
        print("Odometry Data:")
        print("Message Type:", self.odom["msg_type"])
        print("X:", self.odom["x"])
        print("Y:", self.odom["y"])
        print("W:", self.odom["w"])
        print("End:", self.odom["end"])

    def transform_to_cell(self,x,y,resolution):
        self.reso=resolution
        self.cellx = int(x/self.reso)
        self.celly = int(y/self.reso)
        return self.cellx,self.celly
    
    def update_map(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    # Both subscriptions are handled by Localization now
    local = Localization()

    rclpy.spin(local)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

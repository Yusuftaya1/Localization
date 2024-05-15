import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def map_callback(self, msg):
        map_list = list(msg.data)
        map_json = {
            "msg_type": "MAP",
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "data": map_list,
            "end": "*end\n"
        }
        self.map_array = np.array(map_list).reshape(int(map_json["width"]), int(map_json["height"]))

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "*end\n"
        }
        self.update_map(odom_data["x"],odom_data["y"])
    
    def transform_to_cell(self,x,y,resolution):
        self.reso=resolution
        self.cellx = int(x/self.reso)
        self.celly = int(y/self.reso)
        return self.cellx, self.celly 
    
    def update_map(self,X=0,Y=0,R=0.05):
        cellx , celly = self.transform_to_cell(X,Y,R)
        print("cellx: ",cellx)
        print("celly: ",celly)


def main(args=None):
    rclpy.init(args=args)

    localization = Localization()

    rclpy.spin(localization)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

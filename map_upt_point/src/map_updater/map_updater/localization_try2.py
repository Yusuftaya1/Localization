import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
<<<<<<< HEAD
import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
=======

class Localization(Node):
    def _init_(self):
        super()._init_('localization')
>>>>>>> 8d5a401d0637cb29d204b4b7ecd67cd559cdbd1b
        self.map_data = None
        self.odom_data = None
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def map_callback(self, msg):
<<<<<<< HEAD
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
=======
        map_data = list(msg.data)
        map_json = {
            "msg_type": "MAP",
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "data": map_data,
            "end": "*end\n"
        }
        self.map_data = map_json
>>>>>>> 8d5a401d0637cb29d204b4b7ecd67cd559cdbd1b

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "*end\n"
        }
<<<<<<< HEAD
=======
        self.odom_data = odom_data
>>>>>>> 8d5a401d0637cb29d204b4b7ecd67cd559cdbd1b

    def transform_to_cell(self,x,y,resolution):
        self.reso=resolution
        self.cellx = int(x/self.reso)
        self.celly = int(y/self.reso)
        #self.cellw = 
        return self.cellx, self.celly #self.cellw
<<<<<<< HEAD
    
    def update_map(self):
        cellx , celly = self.transform_to_cell(self.odom_data["x"],self.odom_data["y"],self.map_data["resolution"])#,odom_data["w"]
=======

    def update_map(self):
        cellx , celly = self.transform_to_cell(self.odom_data["x"],self.odom_data["y"],self.map_data["resolution"])#,odom_data["w"]
        ## create publisher
        ## map array update procces
        ## publish updated map
    
>>>>>>> 8d5a401d0637cb29d204b4b7ecd67cd559cdbd1b

def main(args=None):
    rclpy.init(args=args)

    localization = Localization()

    rclpy.spin(localization)
    rclpy.shutdown()

<<<<<<< HEAD
if __name__ == '__main__':
=======
if __name__ == '_main_':
>>>>>>> 8d5a401d0637cb29d204b4b7ecd67cd559cdbd1b
    main()
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry

class Localization(Node):
    def _init_(self):
        super()._init_('localization')
        self.map_data = None
        self.odom_data = None
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def map_callback(self, msg):
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

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "*end\n"
        }
        self.odom_data = odom_data

    def transform_to_cell(self,x,y,resolution):
        self.reso=resolution
        self.cellx = int(x/self.reso)
        self.celly = int(y/self.reso)
        #self.cellw = 
        return self.cellx, self.celly #self.cellw

    def update_map(self):
        cellx , celly = self.transform_to_cell(self.odom_data["x"],self.odom_data["y"],self.map_data["resolution"])#,odom_data["w"]
        ## create publisher
        ## map array update procces
        ## publish updated map
    

def main(args=None):
    rclpy.init(args=args)

    localization = Localization()

    rclpy.spin(localization)
    rclpy.shutdown()

if __name__ == '_main_':
    main()
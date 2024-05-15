import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped

class MapSub(Node):
    def __init__(self, localization):
        super().__init__('map_subscriber')
        self.localization = localization
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
        self.localization.set_map_data(map_json)

class OdomSub(Node):
    def __init__(self, localization):
        super().__init__('odom_subscriber')
        self.localization = localization
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "**********end***********\n"
        }
        self.localization.set_odom_data(odom_data)

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_data = None
        self.odom_data = None

    def set_map_data(self, map_json):
        self.map_data = map_json
        self._map_data()

    def set_odom_data(self, odom_data):
        self.odom_data = odom_data
        self._odom_data()

    def _map_data(self):
        print("Map Data:")
        print("Message Type:", self.map_data["msg_type"])
        print("Width:", self.map_data["width"])
        print("Height:", self.map_data["height"])
        print("Resolution:", self.map_data["resolution"])
        print("Data:", self.map_data["data"])
        print("End:", self.map_data["end"])

    def _odom_data(self):
        print("Odometry Data:")
        print("Message Type:", self.odom_data["msg_type"])
        print("X:", self.odom_data["x"])
        print("Y:", self.odom_data["y"])
        print("W:", self.odom_data["w"])
        print("End:", self.odom_data["end"])

def main(args=None):
    rclpy.init(args=args)

    localization = Localization()
    map_sub = MapSub(localization)
    odom_sub = OdomSub(localization)

    executor = MultiThreadedExecutor()
    executor.add_node(odom_sub)
    executor.add_node(map_sub)
    executor.add_node(localization)

    rclpy.spin(executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

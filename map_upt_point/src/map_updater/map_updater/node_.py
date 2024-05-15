import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped
import numpy as np

class MapUpdater(Node):
    def __init__(self):
        super().__init__('map_updater')
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_metadata = MapMetaData()
        # Harita boyutlarını ve çözünürlüğünü belirleyin (örneğin)
        self.map_metadata.width = 112
        self.map_metadata.height = 103
        self.map_metadata.resolution = 0.05
        self.map_metadata.origin.position.x = 0.0
        self.map_metadata.origin.position.y = 0.0
        self.map_metadata.origin.position.z = 0.0
        self.map_metadata.origin.orientation.x = 0.0
        self.map_metadata.origin.orientation.y = 0.0
        self.map_metadata.origin.orientation.z = 0.0
        self.map_metadata.origin.orientation.w = 1.0
        self.occupancy_grid = np.zeros((self.map_metadata.width, self.map_metadata.height), dtype=np.int8)
        self.last_position = None

    def position_callback(self, msg):
    # Gelen konumu alın
        current_position = msg.pose.position
        if self.last_position is None:
            self.last_position = current_position
            return
        # Konum farkını hesapla
        position_diff = np.sqrt((current_position.x - self.last_position.x) ** 2 +
                                (current_position.y - self.last_position.y) ** 2)
        # Yarım metrelik bir adımda konum al
        if position_diff >= 0.5:
            # Yeni pozisyonu harita üzerinde işaretle
            x_index = int((current_position.x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
            y_index = int((current_position.y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)
            # Konumu işaretle
            self.occupancy_grid[x_index, y_index] = 100  # Noktayı işaretlemek için 100 kullanıyoruz
            # Haritayı güncelle
            self.publish_map()
            # Son pozisyonu güncelle
            self.last_position = current_position

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info = self.map_metadata

        map_data = self.occupancy_grid.flatten()
        map_msg.data = list(map_data)

        self.map_publisher.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    map_updater = MapUpdater()
    # Robotun konumunu yayınlayan konu adını 'position_topic' olarak değiştirin
    map_updater.subscription = map_updater.create_subscription(
    PoseStamped,
    'position_topic',  # Robotun konumunu yayınlayan konu adını buraya yazın
    map_updater.position_callback,
    rclpy.qos.qos_profile_sensor_data)  # Örnek bir uyumlu QoS profili

    rclpy.spin(map_updater)
    map_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

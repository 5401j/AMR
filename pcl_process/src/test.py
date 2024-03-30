import rclpy
import time
import os
from sensor_msgs.msg import PointCloud2
from livox_ros2.msg import LivoxCustomMsg
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from sensor_msgs.msg import PointField

class LivoxControllerNode:

    def __init__(self):
        self.node = rclpy.create_node('livox_controller_node')
        self.lidar_ids = ["lidar1", "lidar2", "lidar3", "lidar4"]
        self.current_lidar_id = 0
        self.pc2_publisher = self.node.create_publisher(PointCloud2, 'livox_data', 10)
        self.lidar_subscriber = self.node.create_subscription(LivoxCustomMsg, 'livox_topic', self.lidar_callback, QoSProfile(depth=10))
        self.record_duration = 10  # 每個光達的記錄時間（秒）

    def lidar_callback(self, msg):
        # 訂閱到光達消息時的回調函數
        lidar_id = self.lidar_ids[self.current_lidar_id]
        self.save_point_cloud_to_pcd(msg, lidar_id)

        # 建立下一個光達的Timer
        self.current_lidar_id = (self.current_lidar_id + 1) % len(self.lidar_ids)
        self.create_timer()

    def create_timer(self):
        # 創建一個Timer，在record_duration後觸發
        timer = self.node.create_timer(Duration(seconds=self.record_duration).to_msg(), self.timer_callback)

    def timer_callback(self):
        # Timer觸發時的回調函數，表示一個光達的記錄時間結束
        self.node.get_logger().info(f"Lidar {self.lidar_ids[self.current_lidar_id]} recording finished")

    def save_point_cloud_to_pcd(self, msg, lidar_id):
        # 將點雲數據保存為PCD文件
        # 在此添加根據具體數據格式轉換的代碼
        file_name = f"{lidar_id}.pcd"
        file_path = os.path.join(os.getcwd(), file_name)
        self.node.get_logger().info(f"Saving point cloud to {file_path}")

        # 在這裡添加將點雲數據保存為PCD文件的代碼

def main():
    rclpy.init()
    livox_controller_node = LivoxControllerNode()
    rclpy.spin(livox_controller_node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

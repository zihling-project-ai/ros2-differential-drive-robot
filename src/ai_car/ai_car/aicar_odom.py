#!/usr/bin/env python3
from math import sin, cos
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu

import tf2_ros

# ── 常數定義 ────────────────────────────────────────────────
WHEEL_RADIUS     = 0.033   # 輪子半徑 (m)
WHEEL_SEPARATION = 0.16    # 左右輪距 (m)

class AICAR_ODOM(Node):
    def __init__(self):
        super().__init__('aicar_odom')

        # --- 等待 IMU 訊息 ---
        self.get_logger().info('Waiting for /imu ...')
        self.imu_is_reset = False
        self.imu_ang      = 0.0

        # --- 里程計狀態變數 ---
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # --- Subscribers ---
        # TODO: 建立以下三個 Subscriber，綁定對應的 callback
        #   /imu       (Imu)                → cb_imu,       queue=10
        #   /diff_rad  (Float32MultiArray)  → cb_calc_odom, queue=10
        #   /reset     (Empty)              → cb_reset,      queue=1
        self.create_subscription(Imu, '/imu', self.cb_imu, 10)
        self.create_subscription(Float32MultiArray, '/diff_rad', self.cb_calc_odom, 10)
        self.create_subscription(Empty, '/reset', self.cb_reset, 1)

        # --- Publishers ---
        # TODO: 建立以下一個 Publisher
        #   /odom  (Odometry)  queue=10 
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # --- TF Broadcaster ---
        # TODO: 建立 tf2_ros.TransformBroadcaster 實例，指定給 self.broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Odometry 初始化 ---
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id  = 'base_footprint'

    # ── Subscriber Callbacks ────────────────────────────────

    def cb_reset(self, msg):
        """
        TODO:
        1. 印出 '[ SET ] Odom reset'
        2. 將 imu_is_reset 設為 False
        3. 將 x, y, theta 歸零
        4. 將 imu_ang 歸零
        """
        self.get_logger().info(f'[ SET ] Odom reset')
        self.imu_is_reset = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_ang = 0.0
        self.last_time = self.get_clock().now()

    def cb_imu(self, msg):
        """
        TODO:
        1. 從 msg.angular_velocity.z 讀取角速度，儲存到 self.imu_ang
        2. 將 imu_is_reset 設為 True（表示 IMU 已就緒）
        """
        self.imu_ang = msg.angular_velocity.z
        self.imu_is_reset = True

    def cb_calc_odom(self, msg):
        """
        TODO:
        1. 若 imu_is_reset 為 False，直接 return（IMU 尚未就緒）

        2. 判斷這個時間步是否有轉動：
           若左右輪的差分值都是 0，代表車子靜止不轉，角速度視為 0；
           否則使用 IMU 回報的角速度

        3. 取得當前時間，並計算距離上一次呼叫過了多久（dt，單位秒）；
           若時間差不合理（小於等於 0），直接離開

        4. 將左右輪的弧度差分乘上輪子半徑，換算成各自的實際行進距離

        5. 取左右輪行進距離的平均，除以 dt，得到車體的線速度

        6. 根據線速度、當前朝向角與 dt，計算這個時間步在 x、y 方向的位移，
           以及朝向角的變化量，並累加到目前的位置上

        7. 將更新後的位姿與速度填入 Odometry 訊息並發布：
           位置填 x、y（z 為 0），
           朝向用四元數表示（僅繞 Z 軸旋轉），
           線速度與角速度也一併填入

        8. 廣播 TF 座標變換（odom → base_footprint）：
           平移部分填入目前的 x、y（z 為 0），
           旋轉部分與 Odometry 的四元數相同

        9. 將當前時間記錄為 last_time，供下次計算 dt 使用
        """
        if not self.imu_is_reset:
            return

        
        # Step 2：角速度
        if msg.data[0] == msg.data[1] == 0:
            ang_vel = 0.0
        else:
            ang_vel = self.imu_ang

        # Step 3：時間差
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # Step 4：左右輪行進距離
        left_dist = WHEEL_RADIUS * msg.data[0]
        right_dist = WHEEL_RADIUS * msg.data[1]

        # Step 5：車體線速度
        lin_vel = ((left_dist + right_dist) / 2) / dt
        

        # Step 6：累加位移到目前的位置上
        self.x     += lin_vel * cos(self.theta) *dt
        self.y     += lin_vel * sin(self.theta) *dt
        self.theta += ang_vel * dt

        # Step 7：將更新後的位置與速度填入 Odometry 訊息
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0

        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = sin(self.theta / 2)
        self.odom.pose.pose.orientation.w = cos(self.theta / 2)
        
        # 線速度與角速度
        self.odom.twist.twist.linear.x = lin_vel
        self.odom.twist.twist.angular.z = ang_vel

        # 時間戳記
        self.odom.header.stamp = now.to_msg()

        # 發布 Odometry
        self.odom_pub.publish(self.odom)

        # Step 8： 廣播 TF 座標變換（odom → base_footprint）
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.odom.header.stamp
        tf_msg.header.frame_id = self.odom.header.frame_id
        tf_msg.child_frame_id = self.odom.child_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = self.odom.pose.pose.orientation.x
        tf_msg.transform.rotation.y = self.odom.pose.pose.orientation.y
        tf_msg.transform.rotation.z = self.odom.pose.pose.orientation.z
        tf_msg.transform.rotation.w = self.odom.pose.pose.orientation.w
        self.broadcaster.sendTransform(tf_msg)

        # 9. 將當前時間記錄為 last_time，供下次計算 dt 使用
        self.last_time = now

def main():
    rclpy.init()
    node = AICAR_ODOM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

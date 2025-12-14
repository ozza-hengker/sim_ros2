#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ThrusterManager(Node):
    def __init__(self):
        super().__init__('thruster_manager')
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        # Publisher 3 Thruster
        self.left_pub = self.create_publisher(Float64, '/model/sauvc_bot/joint/thruster_left_joint/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/model/sauvc_bot/joint/thruster_right_joint/cmd_thrust', 10)
        self.heave_pub = self.create_publisher(Float64, '/model/sauvc_bot/joint/vertical_propeller_joint/cmd_thrust', 10)

        # SETTING KEKUATAN (GAIN)
        self.GAIN_SURGE = 60.0   # Kecepatan Maju
        self.GAIN_HEAVE = 200.0   # Kecepatan Naik
        self.GAIN_YAW   = 80.0    # Kecepatan Belok

        self.get_logger().info('Thruster Manager Siap (Differential Mode)!')

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # LOGIKA TANK:
        # Maju = Kiri & Kanan putar depan
        # Belok Kiri = Kanan Maju, Kiri Mundur
        left_val = (linear_x - angular_z) * self.GAIN_SURGE
        right_val = (linear_x + angular_z) * self.GAIN_SURGE
        
        # Kirim ke Kiri
        left_msg = Float64()
        left_msg.data = float(left_val)
        self.left_pub.publish(left_msg)

        # Kirim ke Kanan
        right_msg = Float64()
        right_msg.data = float(right_val)
        self.right_pub.publish(right_msg)

        # Kirim ke Atas/Bawah
        heave_msg = Float64()
        heave_msg.data = msg.linear.z * self.GAIN_HEAVE
        self.heave_pub.publish(heave_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ThrusterManager())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

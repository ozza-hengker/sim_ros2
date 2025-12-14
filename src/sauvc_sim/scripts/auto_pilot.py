#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class AutoPilot(Node):
    def __init__(self):
        super().__init__('auto_pilot')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        
        self.start_time = self.get_clock().now()
        self.warmup_duration = 5.0
        
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.MAX_TURN = 0.15 
        self.ALPHA = 0.05 

        # VARIABEL REM (BRAKING)
        self.mission_complete = False
        self.is_braking = False
        self.brake_start_time = None
        self.BRAKE_DURATION = 1.5   # Mundur selama 1.5 detik
        
        self.gate_seen_close = False
        self.CLOSE_THRESHOLD = 6000 # Ambang batas dianggap "Dekat"

        self.get_logger().info('Auto Pilot V10: ACTIVE BRAKING SYSTEM')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        # 0. FASE STOP TOTAL (Setelah Rem Selesai)
        if self.mission_complete:
            self.publisher_.publish(Twist()) # Kirim 0 terus menerus
            cv2.putText(cv_image, "MISI SUKSES!", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(cv_image, "Posisi Terkunci.", (220, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Mata Robot", cv_image)
            cv2.waitKey(1)
            return

        # 0.5 FASE PENGEREMAN AKTIF (Mundur Dulu)
        if self.is_braking:
            now = self.get_clock().now()
            elapsed_brake = (now - self.brake_start_time).nanoseconds / 1e9
            
            if elapsed_brake < self.BRAKE_DURATION:
                # KIRIM PERINTAH MUNDUR (REM)
                brake_cmd = Twist()
                brake_cmd.linear.x = -0.5  # Mundur dengan tenaga medium
                self.publisher_.publish(brake_cmd)
                
                # Visualisasi
                cv2.putText(cv_image, "REM AKTIF (MUNDUR)!", (80, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            else:
                # Waktu rem habis, sekarang STOP total
                self.is_braking = False
                self.mission_complete = True
            
            cv2.imshow("Mata Robot", cv_image)
            cv2.waitKey(1)
            return

        # 1. WARM UP
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        if elapsed < self.warmup_duration:
            self.publisher_.publish(Twist())
            cd = int(self.warmup_duration - elapsed) + 1
            cv2.putText(cv_image, f"MENUNGGU ARUS: {cd}", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Mata Robot", cv_image)
            cv2.waitKey(1)
            return

        # 2. PROSES VISI
        height, width, _ = cv_image.shape
        center_x = width / 2
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 50, 20]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([170, 50, 20]), np.array([180, 255, 255]))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        cmd = Twist()
        target_x = None
        status = ""
        max_area = 0

        if sorted_contours:
            max_area = cv2.contourArea(sorted_contours[0])

        # LOGIKA DETEKSI "SUDAH LEWAT"
        if max_area > self.CLOSE_THRESHOLD:
            self.gate_seen_close = True 
        
        if not sorted_contours and self.gate_seen_close:
            # Gawang hilang dari pandangan SETELAH dekat -> Picu Rem!
            self.is_braking = True
            self.brake_start_time = self.get_clock().now()
            return # Skip sisa logika, langsung masuk loop rem di frame berikutnya

        # 3. NAVIGASI BIASA
        if len(sorted_contours) >= 2 and cv2.contourArea(sorted_contours[1]) > 100:
            M1 = cv2.moments(sorted_contours[0]); cx1 = int(M1['m10']/M1['m00']) if M1['m00'] else 0
            M2 = cv2.moments(sorted_contours[1]); cx2 = int(M2['m10']/M2['m00']) if M2['m00'] else 0
            target_x = (cx1 + cx2) / 2
            status = "Target Tengah"
            cv2.line(cv_image, (cx1, 240), (cx2, 240), (255, 0, 0), 2)
        elif len(sorted_contours) >= 1 and max_area > 500:
            x, y, w, h = cv2.boundingRect(sorted_contours[0])
            cx = x + w/2
            if cx < center_x: target_x = cx + 150; status = "Hindar Kiri"
            else: target_x = cx - 150; status = "Hindar Kanan"
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
        else:
            status = "Scanning..."
            self.target_yaw = 0.15
            cmd.linear.x = 0.0

        if target_x is not None:
            error_x = center_x - target_x
            self.target_yaw = error_x * 0.0015
            if abs(error_x) < 40: self.target_yaw = 0.0
            
            # Maju
            if abs(error_x) < 100: cmd.linear.x = 0.3 
            else: cmd.linear.x = 0.05
            cv2.circle(cv_image, (int(target_x), 240), 5, (0, 255, 0), -1)

        # SMOOTHING & LIMITER
        if self.target_yaw > self.MAX_TURN: self.target_yaw = self.MAX_TURN
        if self.target_yaw < -self.MAX_TURN: self.target_yaw = -self.MAX_TURN
        self.current_yaw = (self.current_yaw * (1 - self.ALPHA)) + (self.target_yaw * self.ALPHA)
        cmd.angular.z = self.current_yaw

        self.publisher_.publish(cmd)
        cv2.putText(cv_image, f"{status} | Area: {int(max_area)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.imshow("Mata Robot", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AutoPilot())
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

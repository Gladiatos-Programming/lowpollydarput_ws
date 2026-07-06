#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Int32

class FootContactPublisher(Node):
    def __init__(self):
        super().__init__('foot_contact_publisher')
        
        # Publishers
        self.right_pub = self.create_publisher(Int32, '/foot_contact/right', 10)
        self.left_pub = self.create_publisher(Int32, '/foot_contact/left', 10)
        
        # Subscribers
        self.right_sub = self.create_subscription(Contacts, '/contact_right', self.right_callback, 10)
        self.left_sub = self.create_subscription(Contacts, '/contact_left', self.left_callback, 10)
        
        # State Internal (Default 0 = Melayang)
        self.right_touching = 0
        self.left_touching = 0
        
        # Simpan waktu terakhir kali kaki menyentuh tanah
        self.last_right_time = self.get_clock().now()
        self.last_left_time = self.get_clock().now()
        
        # TIMEOUT: Kalau 0.05 detik (50ms) gak ada update, anggap kaki melayang
        self.timeout_sec = 0.05 
        
        # TIMER: Jalan terus di background (Cek status tiap 20ms / 50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("✅ Foot Contact Publisher with Watchdog Ready")

    def right_callback(self, msg):
        # Kalau callback ini terpanggil, artinya PASTI sedang nabrak sesuatu
        if len(msg.contacts) > 0:
            self.right_touching = 1
            self.last_right_time = self.get_clock().now() # Reset jamnya

    def left_callback(self, msg):
        if len(msg.contacts) > 0:
            self.left_touching = 1
            self.last_left_time = self.get_clock().now() # Reset jamnya

    def timer_callback(self):
        now = self.get_clock().now()
        
        # Cek Kaki Kanan: Apakah sudah lewat batas waktu (timeout)?
        diff_right = (now - self.last_right_time).nanoseconds / 1e9
        if diff_right > self.timeout_sec:
            self.right_touching = 0
            
        # Cek Kaki Kiri: Apakah sudah lewat batas waktu (timeout)?
        diff_left = (now - self.last_left_time).nanoseconds / 1e9
        if diff_left > self.timeout_sec:
            self.left_touching = 0
            
        # Selalu publish status terbaru (entah itu 1 atau 0)
        self.right_pub.publish(Int32(data=self.right_touching))
        self.left_pub.publish(Int32(data=self.left_touching))

def main(args=None):
    rclpy.init(args=args)
    node = FootContactPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
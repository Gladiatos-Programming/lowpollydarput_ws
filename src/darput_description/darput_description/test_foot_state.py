#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
class FootStatusMonitor(Node):
    def __init__(self):
        super().__init__('foot_status_monitor')
        
        # State internal
        self.right_touching = 0
        self.left_touching = 0
        self.current_status = "UNKNOWN"
        
        # Subscriber
        self.sub_right = self.create_subscription(
            Int32,
            '/foot_contact/right',
            self.right_callback,
            10
        )
        self.sub_left = self.create_subscription(
            Int32,
            '/foot_contact/left',
            self.left_callback,
            10
        )
        
        self.get_logger().info("👣 Foot Status Monitor Started!")
        self.get_logger().info("   Listening: /foot_contact/right & /foot_contact/left")
        self.print_banner()
    def print_banner(self):
        print("\n" + "="*45)
        print("       DARPUT FOOT CONTACT STATUS")
        print("="*45)
        print("  [0] = Ngambang  |  [1] = Menyentuh Tanah")
        print("-"*45 + "\n")
    def determine_status(self):
        r = self.right_touching
        l = self.left_touching
        
        if r == 1 and l == 1:
            return "🦶🦶 DOUBLE SUPPORT (Dua kaki nempel)"
        elif r == 1 and l == 0:
            return "🦶⬜ RIGHT SUPPORT (Hanya kaki kanan)"
        elif r == 0 and l == 1:
            return "⬜🦶 LEFT SUPPORT (Hanya kaki kiri)"
        else:
            return "⬜⬜ NO SUPPORT / FLIGHT (Terbang!)"
    def update_status(self, source):
        new_status = self.determine_status()
        
        # Hanya print jika status benar-benar berubah
        if new_status != self.current_status:
            self.current_status = new_status
            print(f"[{source}] Status changed:")
            print(f"    Right: {self.right_touching} | Left: {self.left_touching}")
            print(f"    → {new_status}")
            print("-" * 45)
    def right_callback(self, msg):
        self.right_touching = msg.data
        self.update_status("RIGHT")
    def left_callback(self, msg):
        self.left_touching = msg.data
        self.update_status("LEFT")
def main(args=None):
    rclpy.init(args=args)
    node = FootStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutdown requested. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
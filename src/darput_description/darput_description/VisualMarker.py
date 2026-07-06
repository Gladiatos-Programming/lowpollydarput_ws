#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

class VisualTargetInteractive(Node):
    def __init__(self):
        super().__init__('visual_target_interactive')
        
        # Publisher untuk mengirim command ke IK Pinocchio
        self.ik_pub = self.create_publisher(Pose, '/target_pose/right_leg', 10)
        
        # Server Interactive Marker
        self.server = InteractiveMarkerServer(self, "target_controls")
        
        # Posisi Awal (Default)
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0
        self.current_pose.orientation.w = 1.0

        # Buat Markernya
        self.create_interactive_marker()
        
        self.get_logger().info("Interactive Marker Server Ready! Silakan geser dan putar bola di RViz.")

    def create_interactive_marker(self):
        # 1. Setup Marker Utama
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "ik_target_marker"
        int_marker.scale = 0.2
        int_marker.pose = self.current_pose

        # 2. Visualisasi (Bola Merah)
        box_marker = Marker()
        box_marker.type = Marker.SPHERE
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.05
        box_marker.color.r = 1.0
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0

        # Kontrol Visual (Biar bisa dilihat)
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # 3. Kontrol Gerakan (Translasi 3D) dan Orientasi (Rotasi 3D)
        self.add_6dof_controls(int_marker)

        # 4. Apply ke Server
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def add_6dof_controls(self, int_marker):
        # Kontrol untuk pergerakan 3D bebas
        control_3d = InteractiveMarkerControl()
        control_3d.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control_3d.name = "move_3d"
        int_marker.controls.append(control_3d)

        # --- X Axis (Panah Merah / Cincin Merah) ---
        control_x = InteractiveMarkerControl()
        control_x.orientation.w = 1.0
        control_x.orientation.x = 1.0
        control_x.orientation.y = 0.0
        control_x.orientation.z = 0.0
        control_x.name = "move_x"
        control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control_x)

        control_rot_x = InteractiveMarkerControl()
        control_rot_x.orientation.w = 1.0
        control_rot_x.orientation.x = 1.0
        control_rot_x.orientation.y = 0.0
        control_rot_x.orientation.z = 0.0
        control_rot_x.name = "rotate_x"
        control_rot_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control_rot_x)

        # --- Y Axis (Panah Hijau / Cincin Hijau) ---
        control_y = InteractiveMarkerControl()
        control_y.orientation.w = 1.0
        control_y.orientation.x = 0.0
        control_y.orientation.y = 1.0
        control_y.orientation.z = 0.0
        control_y.name = "move_y"
        control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control_y)

        control_rot_y = InteractiveMarkerControl()
        control_rot_y.orientation.w = 1.0
        control_rot_y.orientation.x = 0.0
        control_rot_y.orientation.y = 1.0
        control_rot_y.orientation.z = 0.0
        control_rot_y.name = "rotate_y"
        control_rot_y.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control_rot_y)

        # --- Z Axis (Panah Biru / Cincin Biru) ---
        control_z = InteractiveMarkerControl()
        control_z.orientation.w = 1.0
        control_z.orientation.x = 0.0
        control_z.orientation.y = 0.0
        control_z.orientation.z = 1.0
        control_z.name = "move_z"
        control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control_z)

        control_rot_z = InteractiveMarkerControl()
        control_rot_z.orientation.w = 1.0
        control_rot_z.orientation.x = 0.0
        control_rot_z.orientation.y = 0.0
        control_rot_z.orientation.z = 1.0
        control_rot_z.name = "rotate_z"
        control_rot_z.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control_rot_z)

    def process_feedback(self, feedback):
        # Callback saat marker digeser atau diputar di RViz
        if feedback.event_type == feedback.POSE_UPDATE:
            # Update posisi internal
            self.current_pose = feedback.pose
            
            # Publish ke topic (Sekarang pos & orientasi quaternions terkirim penuh)
            self.ik_pub.publish(feedback.pose)

def main(args=None):
    rclpy.init(args=args)
    node = VisualTargetInteractive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
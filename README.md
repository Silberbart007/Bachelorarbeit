# Mein Roomba-Controller

Dieses Projekt verwendet ROS 2 und Nav2 zur Navigation eines Roomba-Roboters mit einem Multitouch-GUI.

## ⚙️ Notwendige Anpassungen

Wenn man mit dem stage_ros2-Simulator arbeitet:
Öffne oder kopiere die Datei (mit einem neuen Namen) `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` und stelle sicher, dass folgende Parameter gesetzt sind (überall):
```yaml
  base_frame_id: "base_link"
  odom_frame_id: "odom"
  scan_topic: base_scan
  set_initial_pose: true
  initial_pose:
      x: -4.0
      y: -4.0
      z: 0.0
      yaw: 0.785398
  global_frame: map
  topic: /base_scan

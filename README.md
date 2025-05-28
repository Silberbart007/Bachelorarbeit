# 🤖 Roboter Controller mit ROS 2, Nav2 & Multitouch-GUI

Dieses Projekt nutzt **ROS 2** (*Jazzy*) und **Nav2**, um einen Roboter zu navigieren. Die Steuerung erfolgt über ein **Qt-basiertes Multitouch-GUI**. Zur Simulation wird `stage_ros2` verwendet.

---

## 🔧 Konfiguration

### 🗂️ Parameterdatei für Nav2

Kopiere die Standard-Parameterdatei:

    cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml my_nav2_params.yaml

Bearbeite `my_nav2_params.yaml` wie folgt:

---

## 📍 AMCL-Konfiguration

Der `amcl`-Node benötigt folgende Einstellungen:

    amcl:
      ros__parameters:
        base_frame_id: "base_link"
        odom_frame_id: "odom"
        global_frame_id: "map"
        scan_topic: "base_scan"

        set_initial_pose: true
        initial_pose:
          x: -4.0
          y: -4.0
          z: 0.0
          yaw: 0.785398

        min_particles: 1000
        max_particles: 3000
        update_min_d: 0.25
        update_min_a: 0.2
        resample_interval: 3
        transform_tolerance: 1.0

        laser_model_type: "likelihood_field"
        laser_likelihood_max_dist: 2.0
        z_hit: 0.8
        z_rand: 0.2
        z_short: 0.05
        z_max: 0.05
        sigma_hit: 0.2

        alpha1: 0.05
        alpha2: 0.05
        alpha3: 0.05
        alpha4: 0.05
        alpha5: 0.05

Möglicherweise müssen auch die anderen Nodes in der Param noch Zuweisungen für base\_scan und base\_scan.

---

## 🖥️ RViz Visualisierung

### 🧭 Fixed Frame

- Stelle in RViz den **Fixed Frame** auf `map`.

### ➕ Hinzuzufügende Displays

Anzeige: Map
Typ: Map
Topic: /map (oder /global_costmap/static_layer)

Anzeige: LaserScan
Typ: LaserScan
Topic: /base_scan

Anzeige: AMCL Pose
Typ: Pose
Topic: /amcl_pose

Anzeige: TF
Typ: TF
Topic: -

### 🔗 Erwartete TF-Kette

    map → odom → base_link → laser

---

## 🚀 Start des Systems

1. Simulator starten:

    ros2 launch stage_ros2 stage_ros2.launch.py

2. Nav2 mit Parametern starten:

    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true params_file:=/pfad/zu/my_nav2_params.yaml

3. RViz starten:

    ros2 run rviz2 rviz2

---

## 🧪 Troubleshooting

### Map wird nicht angezeigt

- Prüfe, ob `/map` Daten sendet:

      ros2 topic echo /map

- Alternativ in RViz das Topic `/global_costmap/static_layer` verwenden.

### AMCL verliert die Position

- Tritt meist bei schnellen Drehungen auf.
- Lösung:
  - Erhöhe die `alpha*`-Werte in der AMCL-Konfiguration.
  - Stelle sicher, dass der Laserscan korrekt mit der Karte übereinstimmt.
  - Prüfe `base_scan`-Daten:

        ros2 topic echo /base_scan

### TF-Fehler

- Prüfe die Transformationskette:

      ros2 run tf2_tools view_frames

---

## 📡 Relevante Topics

Topic: /map
Typ: nav_msgs/OccupancyGrid
Beschreibung: Die aktuelle Karte

Topic: /amcl_pose
Typ: geometry_msgs/PoseWithCovarianceStamped
Beschreibung: AMCL-geschätzte Position

Topic: /base_scan
Typ: sensor_msgs/LaserScan
Beschreibung: Laserscanner-Daten

Topic: /tf
Typ: tf2_msgs/TFMessage
Beschreibung: Transformationen zwischen Frames

---

## 📝 Hinweise

- Die initiale Pose muss mit der Startposition im Simulator übereinstimmen.

---

## 📦 Abhängigkeiten

- ROS 2 Jazzy
- nav2_bringup
- stage_ros2
- Qt5 

---

## 📍 GUI-Integration (optional)

Die Multitouch-GUI sendet Bewegungsbefehle über eine Qt-Anwendung, die ROS-Nachrichten publiziert. (Mehr dazu in einem separaten README-Abschnitt.)

---

## 📁 Lizenz

Dieses Projekt basiert auf Open-Source-Komponenten und steht unter MIT-Lizenz.


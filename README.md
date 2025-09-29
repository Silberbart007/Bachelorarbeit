Autor: Max Vtulkin
Mail:  silberbart007@users.noreply.github.com    

# Roboter Controller mit ROS 2, Nav2 & Multitouch-GUI

Dieses Projekt nutzt **ROS 2** (*Jazzy*) und **Nav2**, um einen Roboter zu navigieren. Die Steuerung erfolgt über ein **Qt-basiertes Multitouch-GUI**. Zur Simulation wird `stage_ros2` verwendet.

---

## Konfiguration

### Parameterdatei für Nav2

Kopiere die Standard-Parameterdatei:

    cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml my_nav2_params.yaml

Bearbeite `my_nav2_params.yaml` wie folgt:

---

## AMCL-Konfiguration

Der `amcl`-Node benötigt folgende Einstellungen:

    amcl:
      ros__parameters:
        base_frame_id: "base_link"
        odom_frame_id: "odom"
        global_frame_id: "map"
        scan_topic: "base_scan"

        set_initial_pose: true
        initial_pose:
          x: Startposition Roboter x
          y: Startposition Roboter y
          z: Startposition Roboter z
          yaw: Startrotation Roboter yaw

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
Die Gewichte beim FollowPath können angepasst werden, ja nach Wunsch.

---

## Troubleshooting

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

## Hinweise

Für Caveworld:
- Die initiale Pose muss mit der Startposition im Simulator übereinstimmen.
- In der cave.world-Datei sollte man die größe der Map auf 10,10 Stellen anstatt 16,16.
- In der cave.world Datei sollte man die Startposition auf -4,-4 anpassen.

Allgemein Map und amcl korrekt einrichten (Mit stage):
- World Datei erstellen/verändern, am besten genauso bennen wie die gewünschte Map-pgm (oder png)
- stage.launch.py-Datei erstellen, die genau diese world-Datei verwendet
- Pgm-Größe (z.b. 2200x2800) merken, Resolution merken (z.B. 0.05) - steht beides in der gewünschten Map yaml
- Der Origin-Wert aus der Map-yaml sagt aus, wo genau die untere linke Ecke der map ist. Bei origin = (-100, -100) wäre
  es also bei x und y = -100.0. (in Metern / stage-Koordinaten)
- In der World Datei zunächst (Resolution * x-Größe) und (Resolution * y-Größe) der pgm berechnen und unter floorplan bei size
  entsprechend eingeben.
- Stage ausführen und schauen, wo die untere linke Ecke der Map ist im Simulator. Entsprechend fehlende Distanz von x|y merken
  und eintragen bei floorplan pose.
- Startposition des Roboters beliebig wählen, aber gleich in der world und amcl-Konfigurationsdatei
---

## Abhängigkeiten

- ROS 2 Jazzy
- nav2_bringup
- stage_ros2
- Qt5 

---
## 📁 Lizenz

Dieses Projekt basiert auf Open-Source-Komponenten und steht unter MIT-Lizenz.


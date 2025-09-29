# User Manual - HMI Interface (`test_qt_pkg`)

Dieses User Manual erklärt, wie das HMI-Interface genutzt werden kann, welche Launch-Dateien relevant sind und wie man es in der Simulation oder auf dem Robotino-Roboter startet.

---

## Inhaltsverzeichnis

1. [Projektstruktur](#projektstruktur)  
2. [Launch-Dateien](#launch-dateien)  
3. [Simulation starten](#simulation-starten)  
4. [Robotino-Bridge starten](#robotino-bridge-starten)  

---

## Projektstruktur

Das Interface befindet sich im Package `test_qt_pkg`. Die wichtigsten Ordner sind:

- **include** – Header-Dateien  
- **src** – Quellcode (C++/Qt)  
- **ui** – Qt UI-Dateien  
- **launch** – Launch-Dateien für ROS 2  
- **config** – Konfigurationsdateien  

---

## Launch-Dateien

Die wichtigsten Launch-Dateien für die Nutzung sind:

1. **default_sim.launch.py**  
   - Startet die Standard-Simulation.  
   - Funktioniert direkt, wenn `stage_ros2` im Workspace eingerichtet ist.  

2. **bridge_sim_launch.py**  
   - Startet das Interface und die Bridge für die Kommunikation mit dem Robotino-Roboter.  
   - Benötigt zusätzliche Schritte auf dem Roboter (siehe unten).  

---

## Simulation starten

Wenn du das Interface in der Simulation nutzen möchtest:

1. Stelle sicher, dass `stage_ros2` installiert und im Workspace eingerichtet ist.  
2. Öffne ein Terminal im Workspace.  
3. Starte die Simulation mit:

```bash
ros2 launch test_qt_pkg default_sim.launch.py
```
Alle benötigten Nodes werden automatisch gestartet und die Simulation läuft.

## Robotino-Bridge starten
Für den Einsatz auf dem echten Robotino-Roboter:
1. **Auf dem Robotino anmelden**
   1. Roboter einschalten
   2. Mit WLAN verbinden
   3. Mit SSH verbinden:
      ```bash
      ssh robotino@<IP-Adresse>
      ```
      (Passwort eingeben, falls nötig)
2. **ROS-Workspace einrichten**
   Auf dem Robotino:
   ```bash
   # Im tools_ws-Ordner
   source devel/setup.bash

   # Im dev_ws-Ordner
   source devel/seutp.bash --extend
   ```
3. **Bridge starten**
   Im dev_ws-Ordner auf dem Robotino:
   ```bash
   ros2 launch test_qt_pkg bridge_sim_launch.py
   ```
   Das Interface startet und alle notwendigen Nodes werden ausgeführt.


# Developer Manual - HMI Interface (`test_qt_pkg`)

Dieses Developer Manual beschreibt die Architektur und Struktur des HMI-Interfaces, die wichtigsten Klassen sowie Hinweise, wie neue Features hinzugefügt werden können.

---

## Inhaltsverzeichnis

1. [Allgemeine Struktur](#allgemeine-struktur)  
2. [Architektur der Anwendung](#architektur-der-anwendung)  
   - [ROS-Nodes](#ros-nodes)  
   - [GUI-Komponenten](#gui-komponenten)  
3. [Neue Features hinzufügen](#neue-features-hinzufügen)  

---

## Allgemeine Struktur

- **Qt-Version:** 5.15  
- **ROS 2-Version:** Jazzy  
- **Betriebssystem:** Ubuntu 24.04  
- **IDE/GUI-Design:** Qt Creator + Qt Designer  
- **Build-Tool:** GCC + CMake 3.28, gebaut mit `colcon` auf Basis von `ament_cmake`.  

Das Interface ist als separates ROS-Paket umgesetzt und lässt sich einfach in einen ROS-2-Workspace integrieren. Das Paket enthält:

- Quellcode (`src`)  
- Header-Dateien (`include`)  
- UI-Layouts (`ui`)  
- Konfigurationsdateien (`config`)  
- Launch-Dateien (`launch`)  
- Dokumentation  

Beim Kompilieren entstehen mehrere Executables; die Hauptanwendung (gui_main) wird üblicherweise gestartet, weitere können bei Bedarf ausgeführt werden.

---

## Architektur der Anwendung

Die Anwendung besteht grob aus zwei Komponenten:

1. **Qt-basierte GUI-Elemente**  
2. **ROS-basierte Steuerungsklassen**  

### ROS-Nodes

Alle ROS-Nodes erben von `rclcpp::Node` und erfüllen unterschiedliche Aufgaben:

- **RobotNode:** Senden von Steuerbefehlen an den Roboter und Empfangen von Sensordaten  
- **Nav2Client:** Kommunikation mit Nav2-Action-Servern für autonome Steuerungen  
- **CameraPublisher:** Veröffentlichung des aktuellen Kamerabildes  
- **CmdVelRelayNode / TcpRobotListenerNode / TcpVelClientNode:** Kommunikation zwischen Roboter und Standalone-PC  
- **CompressedToRawNode:** Dekompimierung komprimierter Kamerabilder  

### GUI-Komponenten

Alle GUI-Klassen erben von `QWidget`, das zentrale Fenster von `QMainWindow` (`MainWindow`). Wichtige eigene Widget-Klassen:

- **JoystickWidget:** Virtueller Joystick  
- **CustomTouchSliderHorizontal / CustomTouchSliderVertical:** Slider-Steuerung  
- **ObstacleMapWidget:** Anzeige der Umgebungskarte & Roboterposition (Und praktisch alle Features, die mit der Map zu tun haben) 
- **ControlWidgetWrapper:** Fügt Notaus-Buttons hinzu  
- **EgoWidget:** Ego-View Darstellung  
- **LaserMapWidget:** Laser-Map Darstellung  
- **WheelWidget:** Anzeige der Lenkradsteuerung  

Weitere Klassen dienen Testzwecken oder sammeln Daten in Hilfsklassen effizient.

---

## Neue Features hinzufügen

Es gibt zwei Hauptwege, neue Funktionalitäten zu integrieren:

### 1. Neue Widgets in Qt Designer erstellen

1. UI-Datei im `ui`-Ordner öffnen.  
2. Neues Widget platzieren (Button, Slider, Anzeige etc.).  
3. Widget promoten zu eigener `QWidget`-Klasse, wenn eigene Logik benötigt wird.  
4. In der Implementierung (`src`) Slots/Funktionen für das Widget definieren.  
5. Bei Bedarf ROS-Topics/Services in der Klasse abonnieren oder publizieren.  

### 2. Bestehende Widgets erweitern

1. Bestehendes Widget nutzen.  
2. Neue Funktionen implementieren, die z. B. auf `onClick` oder andere Signals reagieren.  
3. Bei Änderungen an der ROS-Kommunikation ggf. entsprechende Node anpassen oder neuen Subscriber/Publisher hinzufügen.  

---

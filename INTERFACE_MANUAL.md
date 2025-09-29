# Interface-Bedienungsmanual - HMI Interface (`test_qt_pkg`)

Dieses Manual beschreibt die Bedienung des HMI-Interfaces, alle Steuerungsmethoden sowie die Visualisierungselemente und deren Parameter.

---

## Inhaltsverzeichnis

1. [Steuerungsmethoden](#steuerungsmethoden)  
   - [Slider (Rotation/Speed)](#slider-rotationspeed)  
   - [Lenkrad](#lenkrad)  
   - [Speed Buttons](#speed-buttons)  
   - [Joystick](#joystick)  
   - [Camera Tap-Control](#camera-tap-control)  
   - [Autonome Steuerungen](#autonome-steuerungen)  
   - [Inertia Control](#inertia-control)  
   - [Zone Control](#zone-control)  
   - [Stopp-Button](#stopp-button)  
2. [Visualisierung](#visualisierung)  
   - [Kamerabild](#kamerabild)  
   - [Hinderniskarte](#hinderniskarte)  
   - [Vektorpfeil (Kamera)](#vektorpfeil-kamera)  
   - [Parkmodus (Kamera)](#parkmodus-kamera)  
   - [Beam (Karte)](#beam-karte)  
   - [Ghost Robot (Karte)](#ghost-robot-karte)  
   - [Speed Trail (Karte)](#speed-trail-karte)  
   - [Collision Warning Border](#collision-warning-border)  
   - [Ego View](#ego-view)  
   - [Laser Map](#laser-map)  

---

## Steuerungsmethoden

### Slider (Rotation/Speed)

- Steuert Geschwindigkeit und Rotation des Roboters.  
- **Parameter:**
  - **Dynamic Lock:** Rotation wird automatisch angepasst.  
  - **Always Lock:** Rotation konstant fixiert.  
  - **Lock Off:** Keine Einschränkungen, volle Kontrolle.  

---

### Lenkrad

- Steuerung des Roboters ähnlich einem Lenkrad.  
- **Parameter:**
  - Verschiedene Styles wählbar, z. B. Formel1, MV.  

---

### Speed Buttons

- Vordefinierte Geschwindigkeitsstufen.  
- **Funktion:** + Reset Rotation auf 0.  

---

### Joystick

- Omni-Drive-Funktionalität für ganzheitliche Bewegungen.  
- Ermöglicht intuitive Steuerung in alle Richtungen.  

---

### Camera Tap-Control

- Steuerung des Roboters durch Tippen auf das Kamerabild.
- Je weiter oben, desto schneller, links und rechts bestimmen die Richtung (aber immer nur vorwärts).

---

### Autonome Steuerungen

- **Draw Path + Point:** Roboter folgt gezeichnetem Pfad oder einzelnen Punkten.  
- **Follow Finger:** Roboter folgt Fingerbewegungen auf dem Display.  

---

### Inertia Control

- Steuerung mit Trägheitssimulation für sanftere Bewegungen.
- Wischen auf der Karte für Richtungsbestimmung.
- Je schneller die Wischbewegung, desto stärker der Stoß.

---

### Zone Control (Prototyp)

- Steuerung über vorgegebene Zonen auf dem Interface.
- Jeder Finger addiert 1 Ecke für das Polygon.
- Ab 1 Finger lässt sich das Polygon skalieren
- Ab 2 Finger lässt sich das Polygon rotieren.
- Roboter fährt bei loslassen der Finger zum Zentrum des Polygons.

---

### Stopp-Button

- Standardisierter Notaus-Button für sofortiges Anhalten des Roboters.  

---

## Visualisierung

### Kamerabild

- Anzeige des aktuellen Kamerabildes des Roboters.  

---

### Hinderniskarte

- Visualisierung der Umgebung und Hindernisse.  
- **Navigation:** Multi-Touch-Gesten möglich:
  - Rotation  
  - Skalierung  
  - Bewegung  

---

### Vektorpfeil (Kamera)

- Zeigt Richtung und Orientierung des Roboters im Kamerabild an.  

---

### Parkmodus (Kamera)

- Spezielle Ansicht zum Parken, visuelle Unterstützung beim Positionieren.  

---

### Beam (Karte)

- Darstellung der Strahlen zur Sensorik oder Sichtlinien.  
- **Parameter:**
  - Anzahl Strahlen (gleichverteilt)  
  - Farbe  

---

### Ghost Robot (Karte)

- Projektion der zukünftigen Roboterbewegung.  
- **Parameter:**
  - Kurvenstärke  
  - Voraussehungszeitraum  
  - Farbe  

---

### Speed Trail (Karte)

- Zeigt Bewegungsverlauf des Roboters auf der Karte.  
- **Parameter:**
  - Lebensdauer des Trails  
  - Farbe  

---

### Collision Warning Border (Karte)

- Rahmenanzeige bei potenzieller Kollision.  

---

### Ego View

- Ego-Perspektive des Roboters.  
- **Parameter:** Zoom-Faktor  

---

### Laser Map

- Darstellung der Laser-Scan-Daten in Echtzeit.  

---

**Ende des Interface-Bedienungsmanuals**

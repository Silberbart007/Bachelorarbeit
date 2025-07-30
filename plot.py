import csv
import os
import sys
import matplotlib.pyplot as plt
from datetime import datetime
from collections import Counter
import matplotlib.image as mpimg

def parse_timer(timer_str):
    # Entfernt das "Timer: " Prefix und wandelt dann um
    try:
        timer_clean = timer_str.replace("Timer: ", "")
        mm, ss_xx = timer_clean.split(":")
        ss, xx = ss_xx.split(".")
        total_seconds = int(mm) * 60 + int(ss) + int(xx) / 100
        # Filter: Werte über 1 Stunde (3600 Sekunden) ignorieren
        if total_seconds > 3600:
            print(f"Warnung: Unplausibler Timerwert {timer_str} (>{total_seconds}s), setze auf 0")
            return 0.0
        return total_seconds
    except Exception as e:
        print(f"Fehler beim Parsen von Timer '{timer_str}': {e}")
        return None


# Kommandozeilenargument: Pfad zur CSV-Datei
if len(sys.argv) < 2:
    print("Fehler: Bitte gib den Dateinamen als Argument an.\nBeispiel: python plot.py log_2025-07-22T11:20:15.csv")
    sys.exit(1)

csv_filename = sys.argv[1]

# Ordner für Plots anlegen, falls nicht vorhanden
plot_dir = "plots"
if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)

timestamps = []
min_distances = []
vel_x = []
vel_y = []
omega_z = []
pos_x = []
pos_y = []
interaction_names_raw = []
interaction_positions_raw = []
timer_seconds = []

with open(csv_filename, "r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        try:
            timestamps.append(datetime.fromisoformat(row["Timestamp"]))
            min_distances.append(float(row["MinDistance [m]"]))
            vel_x.append(float(row["linear velocity (x)[m/s]"]))
            vel_y.append(float(row["linear velocity(y) [m/s]"]))
            omega_z.append(float(row["angular velocity (z) [m/s]"]))
            pos_x.append(float(row["Robot amcl-position (x)[m]"]))
            pos_y.append(float(row["Robot amcl-position (y) [m]"]))
            interaction_names_raw.append(row.get("Interaction names", ""))
            interaction_positions_raw.append(row.get("Interaction positions [pixels]", ""))
            timer_sec = parse_timer(row["Timer [min:s.100ms]"])
            if timer_sec is not None:
                timer_seconds.append(timer_sec)
        except Exception as e:
            print(f"Fehler in Zeile: {e}")

# aktive Zeitbereiche erkennen: sobald sich der Timer merklich verändert
active_periods = []
start_idx = 0

for i in range(1, len(timer_seconds)):
    # Wenn Timer nicht (mehr) ansteigt, endet die Phase
    if timer_seconds[i] <= timer_seconds[i - 1]:
        # Phase beenden
        if start_idx < i - 1:  # Phase mit mind. 2 Messpunkten
            active_periods.append((start_idx, i - 1))
        start_idx = i  # Neue Phase starten

# Letzte Phase am Ende des Arrays hinzufügen, wenn gültig
if start_idx < len(timer_seconds) - 1:
    active_periods.append((start_idx, len(timer_seconds) - 1))


merged_periods = []
if active_periods:
    current_start, current_end = active_periods[0]

    for start, end in active_periods[1:]:
        if start - current_end <= 5:  # Lücke maximal 5 Indizes
            current_end = end
        else:
            merged_periods.append((current_start, current_end))
            current_start, current_end = start, end
    merged_periods.append((current_start, current_end))
else:
    merged_periods = []

min_duration = 1.0  # Sekunde
filtered_periods = []

for start_i, end_i in merged_periods:
    duration = timer_seconds[end_i] - timer_seconds[start_i]
    if duration >= min_duration:
        filtered_periods.append((start_i, end_i))


# Plot 1: Minimale Distanz über Zeit
plt.figure(figsize=(10, 4))
plt.plot(timestamps, min_distances, label="Minimale Distanz [m]")
plt.xlabel("Zeit")
plt.ylabel("Distanz [m]")
plt.title("Minimale Hindernisdistanz über Zeit")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "min_distance_over_time.png"))
plt.close()

# Plot 1.5: Minimale Distanz über Zeit mit Timer
label_spacing = 100.0  # Sekundenschritte für Zwischenlabels
max_dist = max(min_distances)  # y-Max für Textpositionierung

def find_closest_index(timer_seconds, start_i, end_i, target):
    subset = timer_seconds[start_i:end_i+1]
    closest_idx_rel = min(range(len(subset)), key=lambda i: abs(subset[i] - target))
    return start_i + closest_idx_rel

plt.figure(figsize=(20, 8))
plt.plot(timestamps, min_distances, label="Minimale Distanz [m]")

for start_i, end_i in filtered_periods:
    # Gelber Hintergrundbereich
    plt.axvspan(timestamps[start_i], timestamps[end_i], color='yellow', alpha=0.3)

    # Vertikale Linien für Start und Ende
    plt.axvline(timestamps[start_i], color='orange', linestyle='--', alpha=0.7)
    plt.axvline(timestamps[end_i], color='orange', linestyle='--', alpha=0.7)

    # Start- und End-Label
    plt.text(timestamps[start_i], max_dist * 0.95,
             f"{timer_seconds[start_i]:.1f}s", ha='left', va='top',
             fontsize=8, color='black')
    plt.text(timestamps[end_i], max_dist * 0.95,
             f"{timer_seconds[end_i]:.1f}s", ha='right', va='top',
             fontsize=8, color='black')

    # # Zwischenlabels und Linien alle ~2 Sekunden
    # t = timer_seconds[start_i] + label_spacing
    # while t < timer_seconds[end_i]:
    #     mid_i = find_closest_index(timer_seconds, start_i, end_i, t)
    #     plt.axvline(timestamps[mid_i], color='orange', linestyle=':', alpha=0.5)
    #     plt.text(timestamps[mid_i], max_dist * 0.9,
    #              f"{timer_seconds[mid_i]:.1f}s", ha='center', va='top',
    #              fontsize=7, color='black')
    #     t += label_spacing


plt.xlabel("Zeit")
plt.ylabel("Distanz [m]")
plt.title("Minimale Hindernisdistanz mit aktiven Timerphasen")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "min_distance_over_time_with_timer.png"))
plt.close()


# Plot 2: Lineare Geschwindigkeiten über Zeit
plt.figure(figsize=(10, 4))
plt.plot(timestamps, vel_x, label="linear velocity x [m/s]")
plt.plot(timestamps, vel_y, label="linear velocity y [m/s]")
plt.xlabel("Zeit")
plt.ylabel("Geschwindigkeit [m/s]")
plt.title("Lineare Geschwindigkeit über Zeit")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "linear_velocity_over_time.png"))
plt.close()

# Plot 3: Winkelgeschwindigkeit über Zeit
plt.figure(figsize=(10, 4))
plt.plot(timestamps, omega_z, label="angular velocity z [m/s]", color='r')
plt.xlabel("Zeit")
plt.ylabel("Winkelgeschwindigkeit [m/s]")
plt.title("Winkelgeschwindigkeit über Zeit")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "angular_velocity_over_time.png"))
plt.close()

# Plot 3.5 Geschwindigkeit über Zeit
plt.figure(figsize=(10, 4))
plt.plot(timestamps, vel_x, label="linear velocity x [m/s]")
plt.plot(timestamps, vel_y, label="linear velocity y [m/s]")
plt.plot(timestamps, omega_z, label="angular velocity z [m/s]")
plt.xlabel("Zeit")
plt.ylabel("Geschwindigkeit [m/s]")
plt.title("Geschwindigkeit über Zeit")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "velocity_over_time.png"))
plt.close()

# Plot 4: Roboter Trajektorie (x,y)

# Map-Größe in Metern (muss man an reale Karte anpassen)
map_width = 1000 * 0.1  # 100 m
map_height = 500 * 0.1  # 50 m

# Pfad zur PGM-Datei
map_path = "custom_fun_map.pgm"

# Map-Bild laden
map_img = mpimg.imread(map_path)

plt.figure(figsize=(20, 10))  # 2:1 Verhältnis passend zur Karte

# Karte als Hintergrund plotten
plt.imshow(map_img, cmap='gray', extent=[0, map_width, 0, map_height], origin='upper')

# Robotertrajektorie drüber plotten
plt.plot(pos_x, pos_y, marker='o', markersize=2, linestyle='-', color='red', label='Position')

plt.xlabel("Position x [m]")
plt.ylabel("Position y [m]")
plt.title("Roboter-Trajektorie auf Karte")
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(os.path.join(plot_dir, "robot_trajectory_on_map.png"))
plt.close()

# --- Plot 5: Interaktionen über Zeit ---
interactions_per_timestamp = []

for name_str in interaction_names_raw:
    if name_str.strip() == "":
        interactions_per_timestamp.append(0)
    else:
        count = name_str.count("|")
        interactions_per_timestamp.append(count)

plt.figure(figsize=(10, 4))
plt.plot(timestamps, interactions_per_timestamp, label="Interaktionen")
plt.xlabel("Zeit")
plt.ylabel("Anzahl Interaktionen")
plt.title("Anzahl der Interaktionen über Zeit")
plt.grid(True)
# plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "interactions_over_time.png"))
plt.close()

# --- Plot 6: Fingerpositionen auf dem Screen ---
finger_x = []
finger_y = []

for pos_str in interaction_positions_raw:
    entries = pos_str.strip().split('|')
    for entry in entries:
        if entry.startswith("(") and entry.endswith(")"):
            try:
                x_str, y_str = entry[1:-1].split('/')
                finger_x.append(float(x_str))
                finger_y.append(float(y_str))
            except:
                continue

plt.figure(figsize=(16, 9))  # ggf. Displayverhältnis anpassen
plt.scatter(finger_x, finger_y, alpha=0.5, s=10, c='blue')
plt.xlabel("Pixel X")
plt.ylabel("Pixel Y")
plt.title("Fingerpositionen auf dem Bildschirm")
plt.axis("equal")
plt.xlim(0, 3840)
plt.ylim(0, 2160)
plt.gca().invert_yaxis()  # GUI-Koordinaten: Y=0 ist oben
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "finger_positions.png"))
plt.close()

# --- Plot 7: Widget-Häufigkeit ---
widget_counter = Counter()

for name_str in interaction_names_raw:
    widgets = name_str.strip().split('|')
    for widget in widgets:
        if widget:
            widget_counter[widget] += 1

widgets_sorted = sorted(widget_counter.items(), key=lambda x: x[1], reverse=True)
widget_names = [w[0] for w in widgets_sorted]
widget_counts = [w[1] for w in widgets_sorted]

plt.figure(figsize=(10, 5))
plt.bar(widget_names, widget_counts)
plt.xlabel("Widget")
plt.ylabel("Anzahl Interaktionen")
plt.title("Interaktionen pro Widget")
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "widget_interactions.png"))
plt.close()

# Kreisdiagramm
top_n = 5
widget_names = widget_names[:top_n]
widget_counts = widget_counts[:top_n]
plt.figure(figsize=(8, 8))
plt.pie(widget_counts, labels=widget_names, autopct='%1.1f%%', startangle=140)
plt.title("Interaktionen pro Widget")
plt.axis('equal')  # Kreis statt Ellipse
plt.tight_layout()
plt.savefig(os.path.join(plot_dir, "widget_interactions_pie.png"))
plt.close()

print(f"Alle Plots wurden erstellt und im Ordner '{plot_dir}/' gespeichert.")

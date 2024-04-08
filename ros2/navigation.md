# Navigation

Nav2 ist eine Paketsammlung, welche die Implementierung von Navigation in unseren Robotern ermöglicht. Es umfasst Pakete zur:

- Kartenverwaltung (Laden, Speichern und Bereitstellen)
- Lokalisierung
- Pfadplanung
- Sensordatenkonvertierung einer Umgebungswelt

## Setup

### Nav2

Installiere die **ros navigation** Pakete mit dem folgenden Befehl:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Turtlebot3

Ein _TurtleBot_ ist ein beliebter mobiler Roboter, der für Bildungs- und Forschungszwecke entwickelt wurde. Er verfügt über grundlegende Sensoren zur Umgebungs- und Positionsbestimmung. Auf dem Logo dieser Dokumentsammlung ist übrigens eine spielerische Interpretation des _TurtleBots_ dargestellt.

Installiere die notwendigen Pakete zur Simulation und Steuerung eines Turtlebots mit dem folgenden Befehl:

```bash
sudo apt install ros-humble-turtlebot3*
```

### SLAM

SLAM steht für _**S**imultaneous **L**ocalization **a**nd **M**apping_. Es bezeichnet ein Verfahren, bei dem ein Roboter gleichzeitig eine Karte seiner Umgebung erstellt und seine Position darin bestimmt.

Installiere für die Verwendung von SLAM in Nav2 das folgende Paket:

```bash
sudo apt install ros-humble-slam-toolbox
```

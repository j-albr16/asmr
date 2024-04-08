# Linux

## ROS2 Humble Installationsanleitung für Ubuntu 22.04 LTS (Jammy)

Herzlich willkommen zur Installationsanleitung für ROS2 Humble! Wenn du zum ersten Mal mit diesem Thema zu tun hast, keine Sorge, wir werden dich durch den Einrichtungsprozess Schritt für Schritt begleiten.

Diese Anleitung basiert auf der offiziellen [Installationsanleitung für ROS2 Humble auf Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Installation

### Einrichtung der Locales

Zuerst stellen wir sicher, dass deine Locales UTF-8-Encoding unterstützen. Öffne ein Terminal und führe den folgenden Befehl aus:

```bash
locale
```

Wenn UTF-8-Unterstützung nicht aktiviert ist, führe die folgenden Befehle aus:

```bash
sudo apt update &&
sudo apt install locales &&
sudo locale-gen en_US en_US.UTF-8 &&
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 &&
export LANG=en_US.UTF-8
```

Du kannst überprüfen, ob die Einstellungen korrekt sind, indem du erneut `locale` ausführst.

### Aktivierung des Repositories

Um das ROS2-Repository zu aktivieren, führe die folgenden Befehle aus:

```bash
sudo apt install software-properties-common &&
sudo add-apt-repository universe
```

Füge als nächstes den ROS2-GPG-Schlüssel hinzu:

```bash
sudo apt update &&
sudo apt install curl -y &&
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Füge das ROS2-Repository deiner Quellenliste hinzu:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Installation von ROS2

Nachdem die Repositories hinzugefügt wurden, aktualisiere deine Quellenliste und installiere dann die verfügbaren Aktualisierungen:

```bash
sudo apt update &&
sudo apt upgrade
```

Jetzt bist du bereit, ROS2 Humble zu installieren:

```bash
sudo apt install ros-humble-desktop
```

Nach der Bestätigung kann dies nun etwas länger dauern.

## Überprüfung der Installation

Um die Installation zu überprüfen, öffne zwei Terminalfenster und führe die folgenden Befehle aus:

Im ersten Terminalfenster:

```bash
. /opt/ros/humble/setup.bash &&
ros2 run demo_nodes_cpp talker
```

Hier sollte nun eine Ausgabe der folgenden Art erscheinen:
```bash
[INFO] [1710440316.393082760] [talker]: Publishing: 'Hello World: 1'
[INFO] [1710440317.393055666] [talker]: Publishing: 'Hello World: 2'
[INFO] [1710440318.393076166] [talker]: Publishing: 'Hello World: 3'
...
```

Im zweiten Terminalfenster:

```bash
. /opt/ros/humble/setup.bash &&
ros2 run demo_nodes_py listener
```

Hier werden die Nachrichten nun empfangen und man erhält eine Ausgabe in der folgenden Art:
```bash
[INFO] [1710440346.408530216] [listener]: I heard: [Hello World: 31]
[INFO] [1710440347.394168101] [listener]: I heard: [Hello World: 32]
[INFO] [1710440348.394213225] [listener]: I heard: [Hello World: 33]
...
```

Die Prozesse könnt ihr nun mit <kbd>Strg + C</kbd> beenden und bei Bedarf die Terminalausgabe mit `clear` säubern.

## Turtlesim

Turtlesim ist ein einfaches ROS-Paket, das eine virtuelle Schildkrötenrobotersimulation bietet. Es dient dazu, grundlegende ROS-Konzepte wie das Veröffentlichen und Abonnieren von Nachrichten sowie die Steuerung von Robotern zu demonstrieren. Im Folgenden bieten wir einen Teaser zur Verwendung von Turtlesim, einschließlich Installation, Ausführung der Simulation und interaktiver Steuerung der virtuellen Schildkröte. Tauche ein und entdecke die Welt der Robotersimulation mit Turtlesim!

### Installation

Überprüfe, ob das Turtlesim-Paket installiert ist:

```bash
ros2 pkg executables turtlesim
```

```{note}
Falls du dich mittlerweile in einem neuen Konsolenfester befindest, kann es sein, dass bash auf einmal kein `ros2` mehr kennt. Um dies zu beheben, musst du noch einmal neu mit `source /opt/ros/humble/setup.bash` [sourcen](sourcen.md).
Dies kannst du natürlich auch direkt in deine `.bashrc` einbinden.
```

Du solltest die folgende Ausgabe sehen:

```bash
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

Da du bereits das ROS Desktop Paket installiert hast, ist Turtlesim schon auf deinem System vorhanden. Um Turtlesim noch einmal getrennt zu installieren, führe den folgenden Befehl aus:

```bash
sudo apt install ros-humble-turtlesim
```

### Ausführung

Um die Turtlesim-Simulation zu starten, führe folgenden Befehl aus:

```bash
ros2 run turtlesim turtlesim_node
```

### Benutzung

Weitere Befehle müssen nun natürlich wieder in einem weiteren Terminal geschehen, da das letzte mit der Simulation beschäftigt ist. Um mit der Simulation interaktiv interagieren zu können, starte die Turtle-Steuerung mit der Tastatur:

```bash
ros2 run turtlesim turtle_teleop_key
```

Du kannst nun die Turtle mit den Pfeiltasten steuern, solange das aktuelle Terminal fokussiert bleibt. Weitere Steuerungsoptionen erfährst du in der Ausgabe.

## RQT

RQT ist ein Framework zur Entwicklung von grafischen Benutzeroberflächen (GUIs) für ROS. Es bietet verschiedene Plugins, die Funktionen wie das Anzeigen von ROS-Graphen, das Überwachen von ROS-Nachrichten und das Steuern von ROS-Nodes ermöglichen.

Die Installation von RQT ist optional, jedoch kann es für die Entwicklung von ROS-Anwendungen mit einer benutzerfreundlichen Oberfläche hilfreich sein.

Um RQT zu installieren, führe die folgenden Befehle aus:

```bash
sudo apt update &&
sudo apt install ~nros-humble-rqt*
```

## Fertig!

Herzlichen Glückwunsch! Du hast erfolgreich ROS2 Humble auf deinem Ubuntu-System installiert. Jetzt bist du bereit, ROS2 zu erkunden und damit zu entwickeln. Wenn du Fragen hast oder auf Probleme stößt, frag uns einfach, schau in die offizielle ROS2-Dokumentation oder suche Hilfe in der ROS-Community. Viel Spaß beim Coden!
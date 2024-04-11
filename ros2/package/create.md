# Erstelle ein Package

## colcon

Um ein Paket zu bauen, benötigst du zunächst `colcon`. Dies installierst du mit:

```bash
sudo apt install python3-colcon-common-extensions
```

## Generell

Gegebenenfalls musst du nun zunächst wieder deine ROS2 Umgebung [sourcen](../setup/sourcen.md).

Jetzt kannst du das Paket erstellen. Beachte dabei, dass das Paket in dem aktuellen Verzeichnis erstellt wird.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python <package_name> 
```

Nachdem dein Paket erstellt ist, kannst du es mit folgendem Befehl bauen:

```bash
cd ~/ros2_ws
colcon build
```

## Hello World Package

Wir erstellen nun ein weiteres Paket, welches einen _Node_ erstellt, um ein `Hello World` Programm zu starten.

Hierzu geben wir bei dem `ros2 pkg create` Befehl ebenfalls einen Wert für die `--node-name` Option an:

```bash
source /opt/ros/iron/setup.bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python <package_name> --node-name demo_node
cd ~/ros2_ws
colcon build
```

### Ausführen des Hello World Programms

Nach dem Bauen kannst du nun deinen lokalen ROS2 Workspace sourcen:

```bash
source ~/ros2_ws/install/setup.bash
```

Anschließend kannst du den erstellten _Node_ mit folgendem Befehl starten:

```bash
ros2 run demo demo_node
> Hi from demo_node.
```


# Erstelle ein Package

TODO: `colcon` Installation

## Generell

Zuerst musst du deine ros2 Umgebung sourcem

Unter Linux nach der gegebenen Anleitung:

```bash
source /opt/ros/iron/setup.bash
```

Jetzt kannst du das package erstellen. Beachte, dass das Package in dem aktuellen Ordner erstellt wird.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python <package_name> 
```

Nachdem dein Package erstellt ist, kannst du es mit folgendem Befehl bauen:

```bash
cd ~/ros2_ws
colcon build
```

## Hello World Package


Wir erstellen nun ein weiteres package welches ein Node erstellt um ein `Hello World` Programm abzulaufen.

Hierzu geben wir bei dem `ros2 pkg create` command ebenfalls einen Wert für die `--node-name` option an:

```bash
source /opt/ros/iron/setup.bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python <package_name> --node-name demo_node
cd ~/ros2_ws
colcon build
```

### Ausführen des Hello World Programms


Nach dem Bauen kannst du nun deinen lokalen ROS2 Workspace source:

```bash
source ~/ros2_ws/install/setup.bash
```

Anschließend kannst du die erstellte Node mit folgendem Befehl starten:

```bash
ros2 run demo demo_node
> Hi from demo.
```

TODO: Wo kommt die Nachricht her? Da fehlt sicher noch was.







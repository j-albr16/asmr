---
jupytext:
  formats: md:myst
  text_representation:
    extension: .md
    format_name: myst
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Launch Files

Launch files können verschieden Nodes starten und stoppen, sowie verschiedene Events bearbeiten oder auslösen.


## Setup

```bash
source /opt/ros/iron/setup.bash
mkdir laumch
```

## Erstelle ein Launch File

Erstelle folgendes launch file mit dem Namen `launch/turtlesim_mimic.py`:

```bash
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

Dieses launch file erstellt 2 turtlesim Nodes wobei auch 2 Turtlesim Fenster gestartet werden. Anschließend wird ebenfalls eine Turtlesim mimic Node erstellt, welche den Pose Topic von turtlesim1 subscribed und den Geschwindigkeitswert and über den /cmd_vel Topic and Turtlesim2 published. Mehr dazu bei dem [Kapitel zu ROS2 Topics](../topic).

:::{note}

Es muss ebenfalls folgende Zeile zum `package.xml` hinzugefügt werden
```xml
<exec_depend>ros2launch</exec_depend>
```

:::



## Starte ein Launch File


Mit folgendem Befehl startest du ein launch File:

```bash
ros2 launch <launch_file_path>
```

```bash
ros2 launch launch/turtlesim_mimic.py
```

Um die Funktionalität zu testen, kannst du mit folgendem Befehl turtle1/sim bewegen:

```bash
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

beide turtle sims sollten sich nun im Kreis bewegen.




# Launch Files

Launch files können verschieden Nodes starten und stoppen, sowie verschiedene Events bearbeiten oder auslösen.

## Setup

```bash
source /opt/ros/humble/setup.bash
mkdir launch
```

## Erstelle ein Launch File

Erstelle folgendes _launch file_ mit dem Namen `launch/turtlesim_mimic.py`:

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

Dieses _launch file_ erstellt zwei _turtlesim Nodes_ wobei auch zwei Turtlesim Fenster gestartet werden. Anschließend wird ebenfalls eine _Turtlesim mimic Node_ erstellt, welche den _Pose Topic_ von `turtlesim1` _subscribed_ und den Geschwindigkeitswert über den `/cmd_vel` _Topic_ an Turtlesim2 _published_. Mehr dazu bei dem [Kapitel zu ROS2 Topics](../topic).

:::{note}
Es muss ebenfalls folgende Zeile zum `package.xml` hinzugefügt werden
```xml
<exec_depend>ros2launch</exec_depend>
```
:::

## Starte ein _Launch File_

Mit folgendem Befehl startest du ein _launch File_:

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

Beide _turtle sims_ sollten sich nun im Kreis bewegen.

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

# Beispiel

Im folgenden Guide werden einige Beispiele zur Nutzung und Steuerung von ROS2 *Nodes* über die CLI dargestellt. 

:::{note}
Stelle sicher, dass bevor du die Beispiele ausführst eine ROS2 Umgebung [aktiviert](../setup/sourcen.md) ist.
:::

Als Beispiel Umgebung wird hier das *turtlesim* Paket verwendet. Mehr dazu findest du in [folgendem](../topic/turtlesim.md) Kapitel.

## Run Nodes

Wie in dem [Kapitel](../package.md) zu packages erklärt ist, enthält jedes ROS2 python *package* eine `setup.py`, in welchem `executables` definiert werden. Diese führen jeweils eine python Funktion aus, die z.B. eine Node startet.

Mit folgendem Befehl kann eine `executable` eines *packages* ausgeführt werden.

```bash
ros2 run <package_name> <executable_name>
```

Als Beispiel kann man die Turtlesim Simulation starten, indem man die `turtlesim_node` executable ausführt.

```bash
ros2 run turtlesim turtlesim_node
```

## List Nodes

Mit folgendem Befehl können alle aktiven Nodes aufgelistet werden:

```bash
ros2 node list
```

Es sollte folgendes in der Konsole ausgegeben worden sein:

```bash
/turtlesim
```

In einem neuen Terminal führe folgenden Befehl aus, welcher eine Node startet die das steuern der Schildkröte ermöglicht:

```bash
ros2 run turtlesim turtle_teleop_key
```

Wenn der befehl `ros2 node list` erneut ausgeführt wird sollte die node ebenfalls aufgeführt werden:

```bash
/turtlesim
/teleop_turtle
```

## Node Info

Der `ros2 node info` Befehl gibt weitere informationen wie aktuelle Subscriber, Pubisher etc. aus. Im allgemeinen sieht der Befehl wie folgt aus:

```bash
ros2 node info <node_name>
```

Du kannst dir nun mit diesem Befehl weitere Informationen über die `turtlesim` Node anzeigen lassen:


```bash
ros2 node info /turtlesim
```

Es sollte folgende ausgabe zu sehen sein:

```bash
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:

```

## Erstellen einer Node

Du kannst in einem erstellten Package Nodes in Form von Python Klassen erstellen. Folgender Code beschreibt eine Node, die "Hello World!" im Terminal ausgibt. 

:::{note}
Die Nodes werden standardmäßig einem Ordner abgelegt, der den Namen des Packages trägt.
:::

```bash
import rclpy
from rclpy.node import Node


class HelloWorldNode(Node):

    def __init__(self):
        super().__init__('hello_world_node')
        self.get_logger().info('Hello World!')


def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Um die Node dem ROS2 System hinzuzufügen, muss sie in der `setup.py` des Packages hinzugefügt werden. 

```bash
...
console_scripts=[
        'hello_world_node = <package_name>.<file_name>:main',
    ],
...
```

Nun kann die Node mit folgendem Befehl ausgeführt werden:

```bash
ros2 run <package_name> hello_world_node
```

:::{note}
Alternativ kann die Node auch direkt über den Python Interpreter ausgeführt werden. Hierzu muss das Pyhton Paket `rclpy` installiert sein. Es kann mit folgendem Befehl installiert werden:

```bash
pip install rclpy
```

Anschließend kann die Node mit folgendem Befehl ausgeführt werden:

```bash
python <file_name>.py
```
:::


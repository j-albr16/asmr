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

Vorerst muss die ROS2 Umgebung geladen werden:

```bash
source /opt/ros/iron/setup.bash
```

## Run Nodes

```bash
ros2 run <package_name> <executable_name>
```

Um die Turtlesim Node zu starten muss folgender Befehl ausgeführt werden:

```bash
ros2 run turtlesim turtlesim_node
```

## List Nodes

Führe folgenden Befehl aus um alle laufenden Nodes aufzulisten.

```bash
ros2 node list
```

Es sollte folgendes zu sehen sein:

```bash
/turtlesim
```

In einem neuen Terminal führe folgenden Befehl aus, welcher eine Node startet die das steuern der Turtle ermöglicht:

```bash
ros2 run turtlesim turtle_teleop_key
```

Wenn der befehl `ros2 node list` erneut ausgeführt wird sollte die node ebenfalls aufgeführt werden:

```bash
/turtlesim
/teleop_turtle
```

## Node Info

Der `ros2 node info` Befehl gibt weitere informationen wir aktuelle Subscriber, Pubisher etc. aus. Verwendung:

```bash
ros2 node info <node_name>
```

Beispiel:


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


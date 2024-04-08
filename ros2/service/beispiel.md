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

## Setup

### Source die ROS2 Umgebung

```bash
source /opt/ros/iron/setup.bash
```

### Turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

in einem anderen Terminal:

```bash
ros2 run turtlesim turtle_teleop_key
```


## Service Liste

Führe folgenden Befehl im Terminal aus um alle Services zu erhalten:

```bash
ros2 service list
```

Output:

```bash
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```

Zum erhalt der Services mitsamt Typ führe folgenden Befehl aus:

```bash
ros2 service list -t
```

```bash
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
...
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
...
```

```bash
```

## Service Typ

Der Typ eines Services gibt an, wie die Request und die Response strukturiert sind.

```bash
ros2 service type <service_name>
```

```bash
ros2 service type /clear
```

Output:

```bash
std_srvs/srv/Empty
```

Der `Empty` Typ meint, dass der Service call keine Daten sendet.

## Service find

Diese Funktion kann man verwenden um Services mit einem bestimmten Typ zu finden:

```bash
ros2 service find <type_name>
```

```bash
ros2 interface show std_srvs/srv/Empty
```

Output:

```bash
/clear
/reset
```

## Service interface show

Diese Funktion wird verwendet um die Stuktur der Input Argumente heraus zu finden:

```bash
ros2 interface show <type_name>
```

```bash
ros2 interface show std_srvs/srv/Empty
```

Da Empty die Leere Eingabe ist kommt folgt als Ausgabe:

```bash
---
```

anders als z.B. `Spawn`:

```bash
ros2 interface show turtlesim/srv/Spawn
```

Output:

```bash
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```


## Service call

Mit dieser Funktion können wir eine Anfrage an einen Service stellen:

```bash
ros2 service call <service_name> <service_type> <arguments>
```

```bash
ros2 service call /clear std_srvs/srv/Empty
```

Es sollten jetzt keine Linien mehr in dem Tutrlesim Fenster zu sehen sein.

Nun versuchen wir eine Schildkröte an einer Position neue zu setzen:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

Service Response Output:

```bash
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

Es sollte eine Neue Schildkröte an der gewünschten Position sichtbar sein.


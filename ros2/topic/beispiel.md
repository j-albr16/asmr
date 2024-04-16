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

Im folgenden werden einige Beispiele zur Verwendung und Steuerung von ROS2 Topics über die ROS2-CLI gezeigt.

## Setup

Als Beispielumgebung wird Turtlesim verwendet. Es wird davon ausgegangen, dass Turtlesim bereits [installiert](turtlesim.md) ist. 

## Topic List

Um herauszufinden welche Topics verfügbar sind, kann folgender Befehl ausgeführt werden:

```bash
ros2 topic list
```

```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

Um zusätzlich den Datentypen der Topics zu sehen, kann folgender Befehl ausgeführt werden:


```bash
ros2 topic list -t 
```

```bash
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```



## Topic Echo

Um zu erfahren was an einen Topic gepublished wird führe folgenden Befehl aus:

```bash
ros2 topic echo <topic_name>
```

Versuchen wir dies nun am Beispiel von Turtlesim

```bash
ros2 topic echo /turtle1/cmd_vel
```

Bewege nun die Schildkröte mit deinen Pfeiltasten und betrachte die Ausgabe im Terminal. Es sollte in etwa folgendes zu sehen sein:

```bash
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

## Topic Info

Um Informationen wir aktuelle Publisher / Subscriber zu erfahren, kann folgender Befehl ausgeführt werden:

```bash
ros2 topic info /turtle1/cmd_vel
```

```bash
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```

## Topic Rqt

Wir können Rqt verwenden um Topics, Nodes etc. zu visualisieren:


```bash
rqt_graph
```

alternativ:

```bash
rqt
```

Navigiere nun zum Node Graph: `Plugins > Introspection > Node Graph`.

## Topic Interface


Der Datentyp, der von einem Topic verwendet wird, wird von einem Interface festgelegt. 
Wir haben schon die Interfaces zu den einzelnen Topics abgefragt. Wir können nun auch die Struktur eines Interfaces abfragen:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

```bash
# This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```

Mehr dazu findest du in der [Interface Dokumentation](../interface.md).


## Topic Pub

Um daten an einen Topic zu publishen, kann folgender Befehl ausgeführt werden:

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```

Um die Schildkröte zu bewegen führen wir folgenden Befehl aus:

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

```bash
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
```

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Der Unterschied hier ist die Entfernung der --once Option und das Hinzufügen der --rate 1 Option, welche ros2 topic pub anweist, die Nachrichten mit einer Frequenz von 1 Hz zu veröffentlichen.

- `--once`: einmalige ausführung
- `--rate 1`: publish kontinuirlich mit einer Frequenz von 1 Hz




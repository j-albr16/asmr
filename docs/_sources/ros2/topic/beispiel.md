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

### Aktivierung des Environments

```bash
source /opt/ros/iron/setup.bash
```

### Turtlesim

Starte turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

In einem anderen Temrinal starte

```bash
ros2 run turtlesim turtle_teleop_key
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


## Topic List

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

um zusätzlich die interfaces zu bekommen:


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

## Topic Info

Um Informationen wir aktuelle Publisher / Subscriber zu erfahren:

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

und dann `Plugins > Introspection > Node Graph`

## Topic Interface


Der Datentyp, der von einem Topic verwendet wird, wird von einem Interface festegelgt. 
Wir haben jetzt schon die Interfaces du Topics erfahren. Um das Interface zu betrachten:

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


## Topic Pub

Um daten an einen Topic zu publishen

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

The difference here is the removal of the --once option and the addition of the --rate 1 option, which tells ros2 topic pub to publish the command in a steady stream at 1 Hz.

- `--once`: einmalige ausführung
- `--rate 1`: publish kontinuirlich mit einer Frequenz von 1 Hz




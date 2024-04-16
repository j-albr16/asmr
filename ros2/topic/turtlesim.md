
# Turtlesim

Turtlesim ist eine einfache Simulationsumgebung für ROS2. In dieser Umgebung werden eine oder mehrere Schildkröten dargestellt, die sich in einer 2D-Umgebung bewegen können. Die Schildkröten können gesteuert werden, um verschiedene Bewegungen auszuführen. Wenn sich eine Schildkröte Bewegt wird eine Spur hinterlassen, die die Bewegung der Schildkröte darstellt.

## Installation

Die Installation von Turtlesim erfolgt über das ROS2-Paket `ros-humble-turtlesim`. Das Paket kann mit folgendem Befehl installiert werden:

```bash
sudo apt install ros-humble-turtlesim
```

Stelle sicher, dass die ROS2 Umgebung [aktiviert](../setup/sourcen.md) ist.

Das Turtlesim Paket beinhaltet mehrere `executables`, die genutzt werden können. Diese sind zum Beispiel zuständig würd das starten von Turtlesim oder das steuern von Schildkröten. Die verschiedenen `executables` können mit folgendem Befehl aufgelistet werden:

```bash
ros2 pkg executables turtlesim
```

Die Ausgabe des Befehls sollte in etwa so aussehen:

```bash
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

In diesem guide verwenden wir die `executables` `turtlesim_node` und `turtle_teleop_key`. Ersteres startet die Simulation und letzteres ermöglicht es die Schildkröten zu steuern.

## Starten von Turtlesim

Um Turtlesim zu starten, kann die `executable` `turtlesim_node` genutzt werden. Dies kann mit folgendem Befehl gestartet werden:

```bash
ros2 run turtlesim turtlesim_node
```

Es sollte sich ein Fenster mit einer Schildkröte Öffnen.

Um die Schildkröte zu steuern, kann die `executables` `turtle_teleop_key` genutzt werden. Diese kann mit folgendem Befehl in einem **neuen** Terminal gestartet werden:

:::{note}
Stelle sicher, dass die ROS2 Umgebung [aktiviert](../setup/sourcen.md) ist.
:::

```bash
ros2 run turtlesim turtle_teleop_key
```

Die Schildkröte kann nun mit den Pfeiltasten gesteuert werden. Die Schildkröte sollte sich in der Simulation bewegen.

## Bewegen der Schildkröte über ROS2 topics

Die Schildkröte kann auch über ROS2 topics gesteuert werden. Starte hierzu die Simulation. Nun können wir die zur verfügung stehenden *topics* betrachtet werden:

```bash
ros2 topic list
```

Die Ausgabe sollte in etwa so aussehen:

```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

Zum bewegen der Schildkröte verwenden wir den *topic* `/turtle1/cmd_vel`. Dieser *topic* erwartet eine Nachricht vom Typ `geometry_msgs/msg/Twist`. Die Nachricht enthält die lineare und die Winkelgeschwindigkeit der Schildkröte.

Um weitere Informationen zu Nachrichtentypen zu erhalten kannst du das entsprechende [Kapitel](../interface.md) besuchen.


Um die Schildkröte nach vorne zu bewegen kann nun eine Nachricht mit dem Typen `geometry_msgs/msg/Twist` an den *topic* `/turtle1/cmd_vel` gesendet werden. Dies kann mit folgendem Befehl erreicht werden:

```bash
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0}, angular: {z: 0.0}}'
```

Die Schildkröte sollte sich nun nach vorne bewegen und eine Spur hinterlassen.

Um die Schildkröte zu drehen, kann die Winkelgeschwindigkeit geändert werden. Dies kann mit folgendem Befehl erreicht werden:

```bash
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 1.0}}'
```

Die Schildkröte sollte sich nun drehen.

## Wahrnehmung der Umgebung

Die Schildkröte kann auch Informationen über ihre Umgebung wahrnehmen. Hierzu können die *topics* `/turtle1/color_sensor` und `/turtle1/pose` genutzt werden. Der *topic* `/turtle1/color_sensor` gibt die Farbe der Schildkröte zurück und `/turtle1/pose` gibt die Position und die Orientierung der Schildkröte zurück.


## Aussicht

In diesem guide haben wir die Turtlesim Simulation installiert, gestartet und eine Schildkröte über die Tastatur und über ROS2 topics gesteuert. In folgenden guides werden wir die Turtlesim Simulation nutzen, um verschiedene Konzepte von ROS2 zu erlernen.







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

## AMCL and Navigation

In diesem Beispiel wird eine Turtlebot Simulation gestartet. Dabei ist die Map bekannt. Mittels [AMCL](https://roboticsknowledgebase.com/wiki/state-estimation/adaptive-monte-carlo-localization/) wird der Turtlebot lokalisiert.

### Starten der Simulation

Bevor die Simulation gestartet werden kann, muss natürlich zunächst [ROS2 gesourced](../setup/sourcen.md) werden. Danach müssen noch einige Umgebungsvariablen gesetzt werden:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

Nun kann die Simulation mit folgendem Befehl gestartet werden:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

Es sollten die Programme `rviz` und `gazebo` starten. Folgendes sollte in `rviz` zu sehen sein:

![rviz](./base.png)

Nun kann die Lokalisation gestartet werden. Klicke hierfür zunächst auf `2D Pose Estimate` und dann dorthin auf die Karte in `rviz`, wo du den Turtlebot vermutest. Der Pfeil soll schließlich in die vermutete Ausrichtung bzw "Sicht"-Richtung des Turtlebots zeigen.

Es sollte Folgendes zu sehen sein:

![pos](./pos.png)

### Navigation

Um den Roboter zu bewegen. Klicke auf `Nav Goal` und klicke auf die gewünschte Position auf der Karte. Hier entspricht der Pfeil der gewünschten Endausrichtung.

Man sollte nun erkennen, dass sich der Roboter bewegt:

![nav](./nav.png)

## SLAM

Um eine SLAM Simulation zu starten, muss auch hier wieder zunächst die ROS Umgebung gesetzt werden:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
```

Nun kann die Simulation mit folgendem Befehl gestartet werden:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

# Aufgabe


Lies dir das Kapitel zu [Turtlesim](../topic/turtlesim.md) und [Services](beispiel.md) durch und führe die dort beschriebenen Schritte aus. Schreibe anschließend ein Programm, welches die Bewegung der Schildkröte über ROS2 Services so steuert, dass ein möglichst kreatives Bild in der Simulation entsteht.

Verwende vor allem die Services:

- `/turtle1/set_pen` um einen Stift zu setzen
- `/turtle1/teleport_absolute` um die Schildkröte zu teleportieren
- `/turtle1/teleport_relative` um die Schildkröte relativ zu teleportieren
- `/spawn` um eine neue Schildkröte zu spawnen


:::{note}
Diese Aufgabe ist als Übungsaufgabe gedacht, um den Umgang mit ROS2 Services zu trainieren. Die Ergebnisse müssen nicht abgegeben werden, aber sie können gerne in der Übung vorgestellt werden. 
:::

:::{admonition} Tip
Das Programm kann in Python oder BASH geschrieben werden. In BASH kann ein einfaches Skript die ROS2 CLI ansteuern. In Python kann eine ROS2 Node erstellt werden, die die Schildkröte steuert.
:::



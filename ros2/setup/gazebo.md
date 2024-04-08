# Gazebo

In dieser Veranstaltung arbeiten wir mit _Ignition Gazebo_, einer Simulationsumgebung, um unseren Code jederzeit zumindest an einem virtuellen Roboter testen zu können.

Mit der folgenden Anleitung gehen wir sicher, dass die korrekte Version von Gazebo verfügbar ist, um eine reibungslose Zusammenarbeit mit deiner ROS2 Version zu garantieren.

Auch hier hält sich die folgende Installationsanleitung weitgehend an die englische [Vorlage](https://gazebosim.org/docs/fortress/getstarted).

## Installation unter Ubuntu

Hierzu brauchst du in der Konsole nur folgenden Befehl ausführen:

```bash
sudo apt install ros-humble-ros-gz
```

## Ausführen

Nun sollte dein System bereit dazu sein, eine Gazebo Simulation zu starten. Starte diese auf folgende Weise:

```bash
ign gazebo shapes.sdf -v 4
```

Hierbei ist `-v 4` ein optionaler Parameter, mit dem Gazebo dir Fehler-, Warnungs-, Info- und Debugging-Nachrichten anzeigt.

```{note}
Sollte sich das Fenster sehr schnell wieder schließen und die Fehler in der Konsole auf einen Zusammenhang mit der GPU schließen lassen, könnt ihr vorher mit `export LIBGL_ALWAYS_SOFTWARE=1` eine Umgebungsvariable setzen, mit der die Simulation per Software-Renderer zwar langsamer läuft, aber zumindest startet.
```

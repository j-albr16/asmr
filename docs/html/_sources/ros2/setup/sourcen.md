# Sourcen

## ROS2 Installation

Damit ihr ROS2 und seine Anwendungen im aktuellen Terminalfenster verwenden könnt, muss die aktuell zu verwendende ROS2 Installation 'gesourced' werden. Das bedeutet in diesem Fall im Grunde nur, dass ihr eine Liste an Befehlen ausführt, nach denen diese Anwendungen dann der Eingabe bekannt sind.

Dies erfolgt mit dem Befehl

```bash
source /opt/ros/humble/setup.bash
```

oder in der Kurzschreibweise mit

```bash
. /opt/ros/humble/setup.bash
```

## Arbeitsbereiche und Pakete

Jedes mal, wenn ihr euren Arbeitsbereich mit `colcon` neu baut, also nachdem ihr diesem neue Pakete hingezufügt oder Pakete bearbeitet habt, müsst ihr auch diesen Arbeitsbereich neu sourcen. Dies erfolgt (schematisch) mit:

```bash
. ~/<workspace>/install/setup.bash
```

## .bashrc

Unter Linux hast du eine Datei in deinem Home-Ordner, welche die Grundeinstellungen für jedes neu geöffnete Terminalfenster beinhaltet. Einerseits kannst du den obigen Befehl in jedem Terminal erneut ausführen, andererseits besteht damit auch die Möglichkeit dies automatisiert auszuführen.

Dazu fügst du diesen Befehl einfach am Ende der Datei `~/.bashrc` hinzu.

Dies erfolgt über den Editor deiner Wahl oder per
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Alternativ kannst du eigene Befehle für auch in eine getrennte Datei auslagern. Dann muss nur diese Datei in der `.bashrc` gesourced werden.

```{note}
Nachdem du deine `~/.bashrc` bearbeitet hast, muss diese im aktuellen Fenster selbst noch gesourced werden - oder du kannst einfach ein neues Terminalfester öffnen, damit die Änderungen aktiv werden. 

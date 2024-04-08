# Package

Ein Paket ist eine Organisationsstruktur für deinen ROS2-Code. Innerhalb eines Pakets ist der Quellcode definiert, um ROS2-Artefakte wie _Nodes_, _Topics_ und _Services_ zu erstellen.

Pakete bieten auch eine einfache Möglichkeit, ROS2-Code mit anderen zu teilen:

## Minimale Bestandteile eines Pakets

```text
asumr_package/
    package.xml
    resource/asumr_package/
    setup.cfg
    setup.py
    asumr_package/
    src/
```

### Erklärung

- **package.xml**: Metainformationen 
- **resource/_<package_name>_**: Markerdatei für das Paket
- **setup.cfg**: Muss vorhanden sein, wenn das Paket Ausführbares enthält (damit `ros2 run` funktioniert)
- **setup.py**: Build-Skript zum Installieren des Pakets
- **_<package_name>_**: Wird von ROS2-Tools verwendet, um das Paket zu finden

## Arbeitsbereich

Ein ROS2-_Workspace_ (Arbeitsbereich) ist ein Ordner, der alle deine Pakete enthält.

```bash
mkdir -p ~/ros2_ws/src
```

Wenn du neue Pakete erstellst, kannst du diese in den `~/ros2_ws/src` Ordner legen.

```{note}
Der Name deines Arbeitsbereiches muss hierbei nicht `ros2_ws` sein, sondern kann beliebig gewählt werden. Beachte hierbei, dass das Anhängsel `_ws` für _workspace_ jedoch gebräuchlich ist, um Arbeitsbereiche einfach als solche erkennen zu können.
```

## Vertiefung

```{tableofcontents}
```

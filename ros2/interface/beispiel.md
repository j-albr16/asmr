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

In ros können eigene Datentypen für Topics und für Services erstellt werden. Diese heißen `Interfaces`. Für Topics werden die Interfaces in ein `.msg` File geschrieben. Diese werden dann in einen `/msg` Ordner im Paket abgelegt. Für Services verwendet man `.srv` Files welche in dem `/srv` Ordner abgelegt werden. Interfaces können nur in c Paketen gebaut werden.

## Erstellung Interface Paket

Es muss folglich ein c ros Paket erstellt werden:

```
ros2 pkg create --build-type ament_cmake custom_interfaces
```

## Erstellung eines `.msg` Files

Wir erstellen nun ein `.msg` File. In diesen `.msg` Files beschreibt jede Zeile eine Variable mit einem Typ und einem Namen.

Hierbei beschreibt jede Zeile eine Variable und der Typ wird über ein Leerzeichen von dem Namen der Variablen getrennt.

```bash
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```

Beispiel:

```bash
int8 telnumber
bool ledig
```

### Arrays

Es können ebenfalls arrays erstellt werden:

```bash
int8[] some_array
int8[5] 5_ints_array
int8[<=5] bis_zu_5_ints_array

string some_string
string<=5 bist_zu_5_char_string
```

### Standard Werte

Standard werte für Variaben können separiert mit einem Leerzeichen an eine Zeile definiert werden:

```
fieldtype fieldname fielddefault
```

```
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```

### Konstanten

Konstanten programmatisch nicht mehr verändert werden.

Eine Konstante kann wie folgt definiert werden:

```
constanttype CONStANTNAME=constantvalue
```

Beispiel:

```bash
int32 X=3
int32 Y=1
```

## Services


Services werden in sog. `.srv` files, die sich in einem `srv/` Ordner im ROS2 Package befinden.
Diese `.srv` bestehen aus einem request und response teil, welche analog zu den `.msg` types definiert werden. Die beiden Teile werden durch das Zeichen `---` getrennt:

```
string str
---
string str
```


## Abschluss Beispiel

Es können zusätzlich auch andere Interfaces refernziert werden:

```
#request constants
int8 FOO=1
int8 BAR=2
#request fields
int8 foobar
another_pkg/AnotherMessage msg
---
#response constants
uint32 SECRET=123456
#response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```

## Bauen

Damit die Interfaces gebaut bzw. zu dem jeweiligen python / c code umgewandelt werden können, müssen folgende Schritte durchgeführt werden:

1. Das `CMakeLists.txt` muss ergänzt für jedes erstellte `.msg` / `.srv` File ergänzt werden:

Setze die folgenden Zeilen dirket über: `ament_package()`

Angenommen du hättest die folgende Interfaces erstellt:
  - "msg/Num.msg"
  - "msg/Sphere.msg"
  - "srv/AddThreeInts.srv"

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
)
```


2. folgende Abhängigkeiten müssen zum `package.xml` hinzugefügt werden:

```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```



## Verwendung

Möchtest du nun die erstellten Interfaces verwenden, kannst du dies wie folgt erreichen:

1. Füge die folgende Abhängigkeit in dein `package.xml` ein:

```xml
...
<exec_depend>custom_interfaces</exec_depend>
...
```

2. Importiere das Interface in deinem python File:

```python
from custom_interfaces.srv import ExampleInterface

...
```
















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

<<<<<<< HEAD
Im folgnden werden einige Beispiele für die Verwendung von Services in ROS2 gezeigt.

## Setup

Zunächst muss die Umgebung gestartet werden. Schaue nochmal in [folgendes](../setup/sourcen).

### Turtlesim

Um die verschiedene Services exemplarisch zu testen eignet sich das turtlesim Paket. Dieses kann mit folgenden Befehlen gestartet werden:

```bash
ros2 run turtlesim turtlesim_node
```

## Service Liste

Mit diesem Befehl können zur Verfügung stehende Services abgefragt werden.

Führe folgenden Befehl im Terminal aus um alle Services zu erhalten:
=======
Im folgenden Text werden einige Beispiele für die Verwendung von Services in ROS2 gezeigt. Als Umgebung hiergür wird das [Turtlesim](../topic/turtlesim.md) Paket verwendet. Zum starten stelle sicher, dass deine ROS2 Umgebung [aktiviert](../setup/sourcen.md) ist und die `turtlesim_node` gestartet ist.

## Service Liste

Um eine Liste der Services zu erhalten führe folgenden Befehl aus:
>>>>>>> main

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

<<<<<<< HEAD
Zum erhalt der Services mitsamt des Typs kann folgender Befehl verwendet werden:
=======
Zusätlich ist es nützlich, die Typen der Services zu kennen. Dies kann mit dem `-t` Flag erreicht werden:
>>>>>>> main

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

Der Typ eines Services gibt an, wie die Request und die Response strukturiert sind. Um den Typ eines Services zu finden, führe folgenden Befehl aus:

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

Mit dieser Funktion können wir eine Anfrage an einen Service stellen. Die allgemeine Syntax ist:

```bash
ros2 service call <service_name> <service_type> <arguments>
```

<<<<<<< HEAD
Konkret können wir nun den `/clear` Service aufrufen, um die Linien in dem Turtlesim Fenster zu löschen:
=======
Mit folgendem Befehl können wir die Linien in der Turtlesim löschen:
>>>>>>> main

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


<<<<<<< HEAD
## Implementation in Python

Im folgenden wird die minimale Implementation eines ROS2 Services mit Python gezeigt.

:::{note}
Es wird davon ausgegangen, dass das `custom_interfaces` Paket bereits erstellt wurde. Dieses besitzt das Interface `AddTwoInts`:

```bash
int64 a 
=======
## Service Implementierung in Python

Im folgenden Beispiel wird eine Service implementiert, der zwei Zahlen addiert. 

:::{note}
Es wird davon ausgegangen, dass zuvor das interface `AddTwoInts` erstellt wurde. Dieses Interface besitzt folgende Struktur:

```bash
int64 a
>>>>>>> main
int64 b
---
int64 sum
```
:::

<<<<<<< HEAD
### Service Server

Der folgende Code zeigt die Implementation eines Service Servers, der zwei Zahlen addiert.

```python
=======
### Server
```python   
>>>>>>> main
from custom_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
<<<<<<< HEAD

### Service Client

Der folgende Code zeigt die Implementation eines Service Clients, der den Service `add_two_ints` aufruft.

```python
=======
### Client

```python   
>>>>>>> main
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
<<<<<<< HEAD

=======
>>>>>>> main

# Topic

Ein ROS2 (Robot Operating System 2) _Topic_ (Thema) ist ein grundlegendes Kommunikationskonzept in ROS2, das dazu dient, Daten zwischen verschiedenen Komponenten eines Roboters oder einer Robotersystemarchitektur auszutauschen. ROS2 ist ein flexibles und modulares Framework zur Entwicklung von Robotersoftware, und _Topics_ sind eine wichtige Möglichkeit, um Informationen zwischen [_Nodes_](node.md) (Knoten) in einem ROS2-System zu übertragen.

## Schlüsselkonzepte

### [_Messages_](TODO)
   _Topics_ dienen dem Austausch von _Messages_. Nachrichten sind Datenstrukturen, die Informationen übertragen. Sie können einfache Datentypen wie Zahlen oder komplexere Strukturen wie Sensorwerte, Bildinformationen oder Steuerbefehle enthalten. ROS2 unterstützt benutzerdefinierte Nachrichtentypen.

### _Publisher_ und _Subscriber_
   Ein ROS2-System besteht aus _Nodes_, die _Publisher_ und/oder _Subscriber_ für verschiedene _Topics_ sein können. Ein _Publisher_ ist ein _Node_, der Nachrichten zu einem bestimmten _Topic_ sendet, während ein Subscriber ein _Node_ ist, der Nachrichten von diesem Topic empfängt.

### _Topic_-Namen
   Jedes _Topic_ hat einen eindeutigen Namen im ROS2-System. Dieser Name ermöglicht es Knoten, die Nachrichten austauschen möchten, das entsprechende _Topic_ zu identifizieren. Der Name folgt normalerweise einem bestimmten Namensschema, z.B., `/sensor_data` oder `/robot_status`.

### Nachrichten-Publizieren
   Ein _Node_, der Informationen bereitstellen möchte, erstellt einen _Publisher_ für ein bestimmtes _Topic_ und veröffentlicht Nachrichten auf diesem _Topic_. Andere _Nodes_ können dann diese Nachrichten abrufen.

### Nachrichten-Abonnieren
   Ein _Node_, der Informationen benötigt, erstellt einen Subscriber für ein bestimmtes _Topic_ und abonniert dieses _Topic_. Wenn ein _Publisher_ Nachrichten auf diesem _Topic_ veröffentlicht, werden sie automatisch an alle _Subscriber_ weitergeleitet.

### Nachrichten-Synchronisation
   ROS2 ermöglicht auch, dass _Nodes_ zeitlich synchronisierte Nachrichten empfangen können. Dies ist insbesondere in Anwendungen wichtig, in denen zeitliche Koordination erforderlich ist - was bei der Robotik häufg der Fall ist.

Insgesamt ermöglichen ROS2 Topics die Kommunikation und den Datenaustausch zwischen den Komponenten eines Roboters oder Robotersystems auf eine strukturierte und modulare Weise. Dies fördert die Wiederverwendbarkeit von Softwarekomponenten und die Skalierbarkeit von Robotersystemen.

## Vertiefung

```{tableofcontents}
```

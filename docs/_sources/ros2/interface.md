# Interface 

Die ROS2 _Interfaces_ (Schnittstellen) sind eine wichtige Weiterentwicklung im ROS2-Framework, welche dazu dient, die Definition und Verwendung von [_Messages_](TODO) (Nachrichten), [_Services_](service.md) (Diensten) und [_Actions_](TODO) (Aktionen) in einem ROS2-System zu vereinheitlichen, damit diese gemeinsam genutzt werden können.

## Schlüsselkonzepte

### _Messages_, _Services_ und _Actions_
   ROS2 unterstützt _Messages_ für asynchrone Datenübertragung über _Topics_, _Services_ für synchrone Kommunikation und _Actions_ für komplexere, lang andauernde Aufgaben. Jede dieser Kommunikationsformen verwendet spezifische Nachrichtentypen, die zuvor separat definiert wurden.

### Schnittstellen
   Ein ROS2 _Interface_ ist eine strukturierte Definition, die von allen drei Arten gemeinsam genutzt werden kann. Die Verwendung von _Interfaces_ führt zur Wiederverwendung von Datenstrukturen und fördert die Codequalität und -konsistenz.

### _Message-Interface_
   ROS2 ermöglicht die Definition von Nachrichten mithilfe von _Interfaces_. Eine Nachricht kann mehrere Felder enthalten, die aus den Datentypen und Namen bestehen, die in der zugehörigen _Interface_-Definition festgelegt sind. _Message-Interfaces_ sind in ROS2-Schnittstellendateien (`.msg`) definiert.

### _Service-Interface_
   Diese werden verwendet, um sowohl die Anfrage- als auch die Antwortseite eines Dienstes zu definieren. _Service-Interfaces_ werden ebenfalls in Schnittstellendateien (`.srv`) erstellt und können in _Service-Servern_ und _-Clients_ verwendet werden.

### _Action-Interface_
   Aktionen sind komplexere Aufgaben, die mehrere Anfragen und Rückmeldungen beinhalten. Mit _Action-Interfaces_ können sowohl die Zieldefinition als auch das Ergebnis der Aktion spezifiziert werden. _Actions_ werden in ROS2-Schnittstellendateien (`.action`) definiert.

### Wiederverwendbarkeit und Konsistenz
   Der Einsatz von _Interfaces_ ermöglicht es, die gleichen Datenstrukturen für Nachrichten, Dienste und Aktionen zu verwenden. Dies erhöht die Codekonsistenz und -qualität und erleichtert die Pflege von ROS2-Systemen.

## Verwendungsbeispiel

Angenommen, du möchtest die Position eines Roboters über ein ROS2-_Topic_ veröffentlichen und auch einen _Service_ anbieten, um die Position auf Anfrage abzurufen. Mit dem _Interface_-Konzept kannst du eine gemeinsame Schnittstelle definieren:

**`position.srv` (Dienst-Schnittstelle)**:
```text
float64 x
float64 y
float64 theta
```

**`position.msg` (Nachrichten-Schnittstelle)**:
```text
float64 x
float64 y
float64 theta
```

In diesem Beispiel teilen die Dienstdefinition und die Nachrichtendefinition die gleiche Schnittstelle zur Darstellung der Positionsinformation. Dies ermöglicht die Wiederverwendung von Datenstrukturen, was zu konsistentem und wartbarem Code führt.

In deinem ROS2-System würdest du dann einen _Service-Server_ erstellen, der die Positionsdienstanfragen verarbeitet, und einen _Publisher_-Knoten, der die Positionsnachrichten auf einem _Topic_ veröffentlicht.

## Vertiefung

```{tableofcontents}
```


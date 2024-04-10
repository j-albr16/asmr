# Service

_Services_ (Dienste) sind ein weiteres wichtiges Kommunikationsmittel in ROS2, die es ermöglichen, synchrone Anfragen und Antworten zwischen verschiedenen [Knoten](node.md) auszutauschen. Während [_Topics_](topic.md) verwendet werden, um asynchrone Datenströme zu übertragen, dienen _Services_ zur Verwaltung von synchroner Kommunikation zwischen Knoten.

## Schlüsselkonzepte

### _Service_-Nachrichten
   _Services_ verwenden [Nachrichten](TODO) zur Kommunikation, ähnlich wie _Topics_. Jedoch sind _Service_-Nachrichten in der Regel zweigeteilt, bestehend aus einer _Service_-Anfrage (_Request_) und einer _Service_-Antwort (_Response_). Die **Anfrage** enthält Informationen, die an einen Dienstleister (_Server_) gesendet werden, während die Antwort die Daten enthält, die vom Dienstleister zurückgegeben werden. Hierbei gilt zu beachten, dass die Rollenverteilung von _Server_ und _Client_ Situationsabhängig sein kann.

### _Service-Server_
   Ein Knoten, der in der Lage ist, einen Dienst anzubieten, wird als _Service-Server_ bezeichnet. Der _Service-Server_ ist für die Entgegennahme von Anfragen und die Bereitstellung von Antworten verantwortlich. Er wartet auf Anfragen von anderen Knoten und führt die angeforderte Aktion aus, wenn eine Anfrage eingeht.

### _Service-Client_
   Ein Knoten, der eine Anfrage an einen _Service-Server_ sendet und auf die Antwort wartet, wird als _Service-Client_ bezeichnet. Der _Service-Client_ initiiert die Kommunikation, indem er eine Anfrage an den _Service-Server_ sendet und dann auf die Antwort wartet.

### _Service_-Namen
   Jeder _Service_ hat einen eindeutigen Namen, der innerhalb des ROS2-Systems verwendet wird, um auf den _Service_ zuzugreifen. _Service_-Namen sind ähnlich wie _Topic_-Namen strukturiert, z.B., `/get_distance`.

### Synchrone Kommunikation
   Im Gegensatz zu _Topics_, bei denen Daten asynchron ausgetauscht werden, sind _Services_ für synchrone Kommunikation konzipiert. Der _Service-Client_ sendet eine Anfrage an den _Service-Server_ und wartet, bis eine Antwort erhalten wird. Dies ermöglicht es, auf Anfragen und Antworten zeitlich genau abgestimmt zu reagieren.

## Beispiel

Angenommen, ihr habt einen Roboter mit einem _Service_, der die aktuelle Temperatur eines Sensors zurückgibt. Der _Service-Server_ würde die Temperaturanfrage entgegennehmen und die aktuelle Temperatur als Antwort zurückgeben. Der _Service-Client_ initiiert die Anfrage und erhält die Temperaturdaten zurück.

Die Implementierung von _Service-Server_ und _-Client_ in ROS2 ähnelt der von _Publisher_ und _Subscriber_, wobei ihr sowohl _Service_-Nachrichten definiert, als auch _Service-Server_-Knoten und _Service-Client_-Knoten erstellt, um Anfragen zu senden und Antworten zu empfangen. Dies ermöglicht es, synchrone Aufgaben durchzuführen, wie beispielsweise das Anfordern von Sensorinformationen, das Ausführen von Berechnungen und das Erhalten von Ergebnissen in Echtzeit.

## Vertiefung

```{tableofcontents}
```

# VM


Sollten die anderen Anleitung aus welchen Gründen auch immer nicht für dich funktionieren oder du möchtest die Inhalte der Robotikvorlesung einfach etwas strikter von deiner Produktiv-System-Installation trennen, bietet sich auch die Möglichkeit alles in einer virtuellen Maschine zu installieren.

Hierzu empfehlen wir VirtualBox mit dem du eine isolierte Entwicklungsumgebung aufsetzen kannst. Dadurch kannst du dann ruhigen Gewissens experimentieren, ohne dir Gedanken über potenzielle Auswirkungen auf dein Host-Betriebssystem machen zu müssen.

Die Einrichtung einer VirtualBox mit Ubuntu 22.04 für die ROS2-Entwicklung ist relativ einfach und erfordert nur wenige Schritte.

# Installation

Hier können wir die den [Guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox) der Ubuntu-Homepage zur Installation in VirtualBox empfehlen.

Falls bei dir alle Stricke reißen, können wir auch eine vorkonfigurierte virtuelle Maschine zu verfügung stellen, dazu musst du vorher nur [VirtualBox](https://www.virtualbox.org/wiki/Downloads) installieren.

Das fertig konfigurierte VirtualBox Image `ubuntu-jammy-asumr.ova` befindet sich auf Sciebo und kann von dort aus heruntergeladen werden. Am Anfang der Vorlesungen wird mit allen Mitgliedern des Learnweb Kurses der Sciebo Ordner geteil, in dem sich das Image befindet. Sollte der Sciebo Ordner nicht angezeigt werden, wende dich bitte an die Betreuer der Übung.

## Konfiguration

Die Datein kann folgendermaßen in Virtual Box eingespielt werden:

1. Öffne Virtual Box und gehe auf *import*
2. wähle das heruntergeladene `ubuntu-jammy-asumr.ova` File aus.
3. Klicke auf *Weiter* und *Importieren*

Nachdem die virtuelle Maschine importiert wurde, kannst du sie starten und dich mit den folgenden Anmeldedaten einloggen:

Nutzername: asumr
Passwort: asumr


Öffne ein Terminal und starte eine turtlesim node um zu überprüfen ob alles korrekt installiert wurde.




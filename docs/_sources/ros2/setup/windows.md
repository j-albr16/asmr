# Windows

## Einleitung:

In der folgenden Anleitung werden wir euch beim Einrichten von WSL (Windows Subsystem for Linux) unterstützen. Obwohl Windows eine beliebte Plattform für die Entwicklung ist, bietet ROS 2 in einer Linux-Umgebung eine Reihe von Vorteilen, die es zu einer bevorzugten Wahl für Entwickler machen.

### Warum in einer Linux-Konsole entwickeln?

Unter Windows bietet das Windows Subsystem for Linux (WSL) die Möglichkeit, eine vollständige Linux-Umgebung in einem Fenster auf Ihrem Windows-Desktop auszuführen. Für die Entwicklung von ROS 2 bietet eine Linux-Konsole unter WSL viele Vorteile:

1. **Nativer ROS 2-Support:** ROS 2 wurde primär für Linux entwickelt, daher ist die Linux-Umgebung ideal für ROS 2-Entwicklung.

2. **Kompatibilität und Stabilität:** ROS 2 und seine Abhängigkeiten sind auf Linux-Umgebungen optimiert, was eine stabilere und zuverlässigere Entwicklungsplattform bietet.

3. **Ressourcenverwaltung:** Linux bietet bessere Ressourcenverwaltungsfunktionen, die für ROS 2-Anwendungen wichtig sind, insbesondere wenn Sie mit großen Datenmengen arbeiten.

4. **Gemeinschaft und Unterstützung:** Die ROS-Community und viele ROS-Pakete werden hauptsächlich von Linux-Benutzern und Entwicklern unterstützt, was den Zugriff auf Ressourcen und Unterstützung erleichtert.

### Hat die Entwicklung unter WSL auch Nachteile?

Obwohl die Entwicklung einfacher ROS2 Anwendungen unserer Erfahrung nach ausreichend performant unter WSL funktioniert, könnten erweiterte Anwendungsbeispiele wie zum Beispiel verteilte ROS Knoten auf mehreren Systemen Anwender vor unverhältnismäßig hohen Konfigurationsaufwand stellen oder bei komplexen Simulationen unnötig zusätzlichen Berechnungsaufwand gerade für schwächere Systeme darstellen. In diesen Fällen ist eine native [Intallation von Linux](linux.md) empfehlenswert.

## Windows Terminal und Powershell

Um den Einstieg in die ROS 2-Entwicklung unter Windows zu erleichtern, zeigen wir dir zunächst, wie du das Windows Terminal installieren und deine PowerShell aktualisieren kannst.

### Anleitungen:

Der unkomplizierteste Weg dazu ist heutzutage die Nutzung von `winget`. Öffne dazu deine jetzige PowerShell im Administratormodus. Diesen findest du mit einem Rechtsklick auf das Windows-Symbol oder mit <kbd>Win</kbd> + <kbd>X</kbd>.

1. **Sicherstellen, dass `winget` auf dem Rechner verfügbar ist**
   - Gib hierzu in einem Browser deiner Wahl auf
     ```
     aka.ms/getwinget
     ```
     Dies startet automatisch den Download der Installationsdatei des aktuellen WinGet-Clients.
2. **Installation des Windows Terminals:**
    - Um nun Windows Terminal zu installieren, führe in der Eingabeaufforderung folgenden Befehl aus:
      ```Powershell
      winget install Microsoft.WindowsTerminal
      ```
    - Dies ist natürlich optional, du kannst also grundsätzlich mit deiner bisherigen Eingabeaufforderung entwickeln.
    - Alternativ kannst du das Terminal auch über den Store installieren:
        - Besuche den Microsoft Store auf deinem Windows-System.
        - Suche nach "[Windows Terminal](https://apps.microsoft.com/detail/9N0DX20HK701)" und klicke auf "Installieren".
    - Nach Abschluss der Installation kannst du das Windows Terminal über das Startmenü oder ab Win11 auch durch Drücken von <kbd>Win</kbd> + <kbd>X</kbd> und Auswahl von "Windows Terminal" öffnen.

3. **Aktualisierung von PowerShell:**
   - Hierzu führe folgenden Befehl aus:
     ```Powershell
     winget install Microsoft.PowerShell
     ```
   - Bei den meisten ist schon eine Installation von PowerShell vorhanden - in diesen Fällen wird eine Aktualisierung vorgenommen.

Nach der Installation des Windows Terminals und der Aktualisierung von PowerShell sind Sie bereit, ROS 2 unter WSL einzurichten und Ihre ROS 2-Projekte auf einer Linux-Konsole zu entwickeln und zu lernen.

## WSL und Ubuntu 22.04 LTS einrichten

```{tip}
Damit diese bei dir funktionieren, kann es sein, dass du in deinem BIOS/UEFI noch Virtualisierung aktivieren musst. Da dies bei jedem Hersteller etwas unterschiedlich ist, können wir dazu nur diesen Hinweis bereitstellen.
```

Um Windows Subsystem for Linux (WSL) mit Ubuntu 22 LTS einzurichten, reicht mittlerweile im Grunde auch eine Zeile. Öffne dazu PowerShell im Administratormodus und führe folgende Zeile aus:

```PowerShell
wsl --install -d Ubuntu-22.04
```

**Alternativ: Ubuntu 22 LTS aus dem Microsoft Store herunterladen:**
   - Öffne den Microsoft Store und suche nach "Ubuntu 22.04 LTS" _oder_ verwende diesen [Link](https://apps.microsoft.com/detail/9PN20MSR04DW).
   - Klicke auf "Installieren", um Ubuntu 22 LTS herunterzuladen und zu installieren.
   - Hier kann es sein, dass du beim ersten Starten noch Hinweise bekommst, wie du WSL noch einrichten musst, um die Distro starten zu können.

```{note}
Hast du bereits eine andere Distribution auf deinem System oder willst generell mehrere Versionen parallel verwalten, hilft die [Anleitung von Microsoft](https://learn.microsoft.com/de-de/windows/wsl/install).
```

Auch bietet MS gute [Empfehlungen](https://learn.microsoft.com/de-de/windows/wsl/setup/environment) zur Einrichtung vieler Programme unter WSL, von unserer Seite zunächst aber nur die wichtigsten ersten Schritte:

1. **Neustart des Systems:**
   - Starte zunächst deinen Computer neu, um ggf. ausstehende Änderungen wirksam werden zu lassen.

2. **Initialisieren von Ubuntu:**
   - Öffne das installierte Ubuntu 22 LTS über das Startmenü oder suche danach.
   - Die erste Ausführung erfordert einige Minuten für die Installation.
   - Nach der Installation wirst du zur Erstellung eines neuen Benutzernamens und Passworts aufgefordert. Gib diese Informationen ein.

3. **Aktualisieren Sie das Ubuntu-Paketrepository:**
   - Generell eine gute Angewohnheit nach jedem Start des Systems.
   - Gib im Terminal folgenden Befehl ein, um das Paketrepository zu aktualisieren und vorhandene Pakete zu aktualisieren:
     ```bash
     sudo apt update && sudo apt upgrade
     ```

5. **Arbeiten mit WSL und Ubuntu:**
   - Starte WSL (Ubuntu 22 LTS) jederzeit, indem du "Ubuntu" im Startmenü suchst und darauf klickst.
   - Du kannst nun Linux-Befehle in der Ubuntu-Shell ausführen und Linux-Anwendungen installieren und verwenden, als ob Sie auf einem Linux-System arbeiten würden.

Mit diesen Schritten sollte WSL mit Ubuntu 22 LTS erfolgreich eingerichtet sein. Du kannst nun Linux-Befehle ausführen und die Linux-Umgebung auf deinem Windows-System verwenden. Damit kannst du nun bei der Anleitung zur [Installation von ROS2 unter Linux](linux.md) fortfahren.

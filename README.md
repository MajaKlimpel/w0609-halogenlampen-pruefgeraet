# W0609 Halogenlampen-Prüfgerät

Prüfgerät für Lebensdauertests an Halogenlampen mit zehn parallel betriebenen Kanälen.

Entstanden im Rahmen der Bachelorarbeit "Konzeption, Entwicklung und Validierung eines
automatisierten Prüfgeräts zur parallelen Prüfung von Halogenlampen" von Maja Klimpel,
Technische Universität Berlin, Fachgebiet Elektronische Mess- und Diagnosetechnik,
in Zusammenarbeit mit der KNAUER Wissenschaftliche Geräte GmbH, 2026.

## Überblick

Halogenlampen werden bei KNAUER als Lichtquellen in HPLC-Detektoren eingesetzt.
Mit dem entwickelten Prüfgerät können bis zu zehn Lampen gleichzeitig unter definierten
Bedingungen getestet werden. Dazu werden wiederkehrende Schaltzyklen mit Anstiegsrampe,
Haltephase, Abwärtsrampe und Abkühlphase durchgeführt. Während des Tests werden Strom
und Spannung jedes Kanals, sechs Temperaturen und die Lüfterdrehzahlen sekündlich
erfasst und gespeichert. Sinkt der Strom eines Kanals während der Haltephase unter
0,5 A, wird die entsprechende Lampe als ausgefallen erkannt. Der betroffene Kanal wird
daraufhin abgeschaltet und der Ausfall zusammen mit der Zyklusnummer gespeichert.

Das System besteht aus zwei Teilen. Auf einem Teensy 4.1 läuft die Firmware, die die
Ausgangsspannungen regelt, den Testablauf steuert und die Messwerte erfasst. Auf dem
PC läuft eine in Python geschriebene Bedienoberfläche, die das Gerät über USB steuert,
die Messwerte darstellt und als CSV-Datei speichert.

## Inhalt des Repositories

```
Elektronik/
  Altium/                Altium-Projekt der Leiterplatte und Schaltplan als PDF
  Fertigungsdaten/       BOM, Bohrdaten, Gerber und Bestückungsdaten
Konstruktion/
  Konstruktionsdaten/    Einzelteile und Baugruppen der mechanischen Konstruktion
  w0609_step_compressed.stp   Gesamtbaugruppe als STEP-Datei
Software/
  Firmware/
    Gesamtsystem/        Firmware des Prüfgeräts (Teensy 4.1)
    Testprogramme/       Testprogramme zur Inbetriebnahme der Teilfunktionen
    Kommunikationsprotokoll_W0609_v2.0.pdf
  GUI/
    W0609_GUI.py         Quellcode der Bedienoberfläche
    W0609 Halogenlampen-Prüfgerät.exe   ausführbare Windows-Anwendung
    Log/                 aufgezeichnete Messreihen
Datenblaetter und Pruefprotokoll/
  Halogenlampe/          Datenblätter der geprüften Lampen
  Deuteriumlampe/        Datenblatt der Deuteriumlampe
  ASA/                   Werkstoffdatenblätter des verwendeten Filaments
  Prüfprotokoll.pdf      Prüfprotokoll des entwickelten Geräts
```

## Elektronik

Der Ordner `Elektronik/Altium` enthält das vollständige Altium-Projekt der entwickelten
Leiterplatte. Der Schaltplan liegt zusätzlich als PDF vor. Unter
`Elektronik/Fertigungsdaten` befinden sich die zur Fertigung und Bestückung erzeugten
Daten (Stückliste, Gerber- und Bohrdaten sowie Bestückungsdaten).

## Konstruktion

Die mechanische Konstruktion liegt als Gesamtbaugruppe im STEP-Format vor
(`w0609_step_compressed.stp`). Der Ordner `Konstruktionsdaten` enthält zusätzlich die
Einzelteile und Baugruppen des CAD-Modells.

## Firmware

Die Firmware unter `Software/Firmware/Gesamtsystem` steuert das Prüfgerät. Sie regelt
die zehn Ausgangsspannungen, steuert die Lüfter und das Gerätedisplay und führt den
Prüfablauf auch ohne angeschlossenen PC weiter. Die Kommunikation mit der 
Bedienoberfläche erfolgt über USB-Serial mit 115200 Baud. Das verwendete ASCII-Protokoll 
ist in Kommunikationsprotokoll_W0609_v2.0.pdf beschrieben. Die Kalibrierwerte der Kanäle 
liegen im EEPROM des Mikrocontrollers und bleiben damit im Gerät erhalten.

Der Ordner `Testprogramme` enthält kleine, voneinander unabhängige Programme, mit denen
die einzelnen Teilfunktionen der Leiterplatte in Betrieb genommen und geprüft wurden:
Buck-Wandler und Enable-Signale, DAC-Ansteuerung, Temperatursensoren, Strom- und
Spannungsmessung über die Multiplexer, Lüfteransteuerung mit Drehzahlrückmeldung sowie
Display, Taster und Status-LED. Jedes Programm spricht nur die jeweilige Baugruppe an
und gibt die Ergebnisse über die serielle Schnittstelle aus.

## Bedienoberfläche

Die Oberfläche ist in Python mit PyQt5 umgesetzt. Sie bietet die Einstellung der
Prüfspannung von 0 bis 7 V gemeinsam oder je Kanal, die Vorgabe von Rampenzeit,
An-Zeit, Aus-Zeit und Zyklenzahl, Live-Diagramme für Strom, Spannung und Temperatur, 
die Auswertung mehrerer Messreihen nach Ausfällen je Zyklus, einen Assistenten zur 
Kalibrierung der Ausgangsspannung und einen Demo-Modus, der die Oberfläche mit 
simulierten Werten auch ohne angeschlossenes Gerät bedienbar macht.

Für den Betrieb wird die Anwendung `W0609 Halogenlampen-Prüfgerät.exe` im Ordner
`Software/GUI` gestartet, der COM-Port des Geräts ausgewählt und auf Verbinden geklickt.
Nach Eingabe der Testparameter und der Batch-Nummer beginnt der Test. Die Messreihe wird
am Ende automatisch im Ordner `Log` neben der Anwendung gespeichert.

Wer den Quellcode direkt ausführen möchte, benötigt Python 3 mit den Paketen PyQt5,
pyserial, matplotlib und numpy, für die Wärmebilddarstellung zusätzlich scipy:

```
python Software/GUI/W0609_GUI.py
```

## Messdaten

Die Dateien im Ordner `Software/GUI/Log` enthalten die aufgezeichneten Messreihen. Sie
sind mit Semikolon getrennt und verwenden das Komma als Dezimaltrennzeichen. Dadurch 
können sie direkt in Excel geöffnet werden. Am Dateianfang stehen Testdatum und 
Batch-Nummer. Jede weitere Zeile entspricht einer Sekunde Messzeit und enthält 
Zeitstempel, Betriebszustand, Zyklusnummer, Strom und Spannung aller zehn Kanäle, 
die sechs Temperaturen, die Lüfterdrehzahlen und die bis dahin ausgefallenen Kanäle.

## Datenblätter und Prüfprotokoll

Der Ordner enthält die von KNAUER bereitgestellten Datenblätter der 
Halogenlampen und der Deuteriumlampe. Außerdem sind dort die Werkstoffdatenblätter
des für die Konstruktion verwendeten Filaments sowie das Prüfprotokoll des
entwickelten Geräts abgelegt.

## Autorin

Maja Klimpel, 2026

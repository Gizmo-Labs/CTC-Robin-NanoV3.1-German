# CTC-Robin-NanoV3.1-German
Spezielle Marlin-Firmware für CTC 3D-Printer modifiziert für MakerBase Robin Nano V3.1 Hauptplatine

![Display](./Dokumentation/Bilder%20Druckermodifikationen/20250120_205207.jpg)


## Hintergrund
Eines Tages gab die originale Hauptplatine meines "CTC-Sperrholzdruckers" ihren Geist auf.<br>
Leider gab es diese nicht mehr als Ersatzteil, sodass ich mir überlegte, was ich nun machen soll.<br>
Weil ich schon viel Mühe und Kapital in die Optimierung des Gerätes gesteckt hatte, und ich mit den <br>Druckergebnissen zufrieden war, bestellte ich mir das MakerBase Robin Nano V3.1 Board.

- [Hier gibt es die Hauptplatine](https://www.roboter-bausatz.de/p/mks-robin-nano-v3-ts35-3d-drucker-mainboard?srsltid=AfmBOoqWtMm4qKDmvziqhnAerP709jFHnOgZVKsleq41IHb9FodTlrFz)

- [Hier sind meine Motortreiber (aktuell nicht verfügbar)](https://shop.watterott.com/SilentStepStick-TMC2208-verloetet)

- [Wahrscheinlich gehen auch diese hier (von mir nicht getestet)](https://www.roboter-bausatz.de/p/5x-bigtreetech-tmc2208-v3.0-schrittmotortreiber-step-dir)

- [Hier ist der Temperatursensor für den Extruder mit 100 kOhm](https://www.cr3d.de/produkt/thermistor-geschraubt-m3-atc-semitec-104gt-2/)

- [Hier ist der Sensor für das Heizbett mit 100 kOhm](https://www.roboter-bausatz.de/p/thermistor-ntc-3950-100k-ohm-mit-1m-anschlusskabel?srsltid=AfmBOorO2iiI2VsnCeCiLfR_8YGiBmcU60Q-6lt0RnoAsfWkLrs_Vr0i)

- [Diese Endschalter verwende ich...](https://www.roboter-bausatz.de/p/optischer-endschalter-mit-kabelsatz-3-pin-50cm-fuer-cnc-3d-drucker-oder-tueren)

## Hinweis
- ⚠️️ **Alle Temperatursensoren müssen auf 100 kOhm geändert werden** ⚠️
- ⚠️️ **Die originalen Temperatursensoren vom CTC-Drucker können nicht verwendet werden** ⚠️  
- ⚠️️ **Diese Firmware ist konfiguriert für einen Drucker mit:**  
📌 Einem Extruder mit 24V Heizpatrone  
📌 Einem Heizbett (ich verwende das originale von CTC)    
📌 Einem Extruder-Lüfter (ich empfehle einen Papst-Lüfter)  
📌 Einem Bauteil-Lüfter (ebenfalls Papst-Lüfter)  
📌 Motortreibern vom Typ TMC2208 (und nur diese...)  
📌 Keinem Bed-Leveling-Sensor   
📌 Extruder-Temperatursensor mit 100 kOhm  
📌 Heizbett-Temperatursensor mit 100 kOhm  
📌 Display TS35 und dem Drehencoder an der Seite (und nur dieses...)  
📌 Übertragung der Druck-Dateien über USB (und nur so...)

## WiFi-Anbindung des Druckers
- [Mit MKS Robin WiFi Modul...](https://www.roboter-bausatz.de/p/mks-robin-tft-wifi-modul?srsltid=AfmBOopBdXZmDU004fd81v3fnUxSQhNalMm-QTwlBtQZSpPEPU3y1NKb)  
- Dazu gibt es eine App, die sich mit dem Modul verbindet.  
- Problem: Es spricht mit der chinesischen Datenkraken-Cloud...  
- Dafür Plug-and-Play und kompatibel mit dem Board !  


## Nützliche Features gegenüber dem original CTC

📌 5-Punkt Druckbettvermessung (es ist eine Wohltat...)  
📌 Babystep (Anpassung Z-Höhe) während des Druckes. Ideal bei Haftungsproblemen.  
📌 Temperaturänderungen während des Druckes möglich.  
📌 Lüfteranpassung während des Druckes möglich.  
📌 Sehr genaue Druckzeitanzeige.  
📌 Verschiedene Alarme (Aufheizen zu langsam, Fühlerbruch etc...)

## Zu dieser Firmware
- ⚠️️ **Diese Firmware ist NUR für einen CTC-Drucker**⚠️
- ⚠️️ **Aus dem geforkten Marlin-Repo wurde fast alles entfernt was nichts mit diesem Drucker zu tun hat.** ⚠️  
- ⚠️️ **Alle "China-Pling-Pling" Icons wurden entfernt, und gegen Material-UI-Icons getauscht.** ⚠️
- ⚠️️ **Personen ohne Erfahrung sollten nichts am Projekt ändern. Es ist komplex.** ⚠️
- ⚠️️ **Der Drucker läuft mit dieser Firmware und den Watterot Treibern sehr schön.** ⚠️
- ⚠️️ **Die Ansteuerung der Motortreiber erfolgt über UART**⚠️
- 📌 **Alles auskommentierte könnte man auch noch entfernen, ich hatte aber keine Lust mehr**

## Projekt kompilieren

Der nachfolgende Screenshot zeigt wie das Projekt kompiliert wird.  
Das gesamte Projekt bezieht sich ausschliesslich auf die rot markierte Konfiguration.

![Build](./Dokumentation/Bilder%20Handhabung%20Softwareprojekt/Build%20Prozess.jpg)

## Zum Thema G-Code Dateivorschau im Display

Die grafische Vorschau des zu druckenden Teiles im Display ist offenbar unter dem CURA-Slicer konfigurierbar.  
Da ich diesen nicht nutze, und dies unnötige Spielerei aus meiner Sicht ist, kann ich dazu nichts sagen.

## Anpassung von Icons + Bildumwandlung in das .bin-Format

- Öffne den [LVGL Online Image Converter hier!](https://lvgl.io/tools/imageconverter)  
- Wähle LVGL v8
- Öffne deine BMP-Bilddatei.
- Wähle bei Color-Format "CF_TRUE_COLOR".  
- Wähle Output-Format "Binary RGB565".
- Starte die Umwandlung.
- Du erhältst eine Datei mit dem gleichen Namen im .bin-Format.

![Converter](./Dokumentation/Bilder%20Handhabung%20Softwareprojekt/Image-Converter.png)  

- Kopiere die Datei in den Ordner "assets".

![Assets](./Dokumentation/Bilder%20Handhabung%20Softwareprojekt/Assets-Ordner.png)  

- ⚠️️ **Alle Icons sind im "assets-Ordner"**  
- ⚠️️ **Um welches Icon es sich handelt lässt der Dateiname erahnen**  
- ⚠️️ **Die .bin-Dateien lassen sich leider nicht anschauen**
- ⚠️️ **Eine Rückführung vom .bin-Format nach Bitmap ist nicht möglich**  
- ⚠️️ **Ich habe nur die Icons angepasst, die ich in der GUI sah**


## Firmware + GUI einspielen

1. Kompiliere das Projekt wie beschrieben
2. Wenn du das Projekt erfolgreich kompiliert hast (und nur dann...)  

- Findest du das Verzeichnis `.pio\build\mks_robin_nano_v3_usb_flash_drive_msc`.  
- Kopiere den Ordner `assets` und die Datei `Robin_nano_v3.bin` auf eine Micro-SD-Karte.

![Kompiliert](./Dokumentation/Bilder%20Handhabung%20Softwareprojekt/Kompiliert.png)  

- Stecke die Micro-SD-Karte in den Slot mit der Beschriftung "Hier Firmware-Update + GUI".  
- Starte die Hauptplatine und die Firmware wird automatisch EINMAL geladen.
- Das User-Interface wird nun ebenfalls aktualisiert.
- Du kannst nun die Micro-SD-Karte wieder entfernen.

![SD-Karte](./Dokumentation/Bilder%20Hauptplatine/Sensorbelegung%20Nano%20V3.1%20+%20CTC.jpg)  


## Hier das offizielle Wiki zur Platine

- [MKS Robin Nano V3.x Wiki](https://github.com/makerbase-mks/MKS-Robin-Nano-V3.X/wiki).


##  Weiterführende Informationen zum Robin Nano V3.X

- [MKS Robin Nano V3 github](https://github.com/makerbase-mks/MKS-Robin-Nano-V3.X).

## Herstellerangaben
- MKS Github: https://github.com/makerbase-mks  
- MKS Facebook: https://www.facebook.com/Makerbase.mks/  
- MKS Twitter: https://twitter.com/home?lang=en  
- MKS Discord: https://discord.gg/4uar57NEyU
- MKS Reddit: https://www.reddit.com/user/MAKERBASE-TEAM/ 

![mks_link](https://user-images.githubusercontent.com/12979070/149611691-1b73b40d-5d51-45b6-9a3f-3fc2281119ff.png)




# Acht Tage auf zwei Rädern

## Der erste Kontakt

Am 1. November 2026 stand ein kleiner Roboter mit Raspberry Pi und 2 GB RAM vor uns, und wir stellten ihm eine einfache Aufgabe: Schau durch deine Kamera, erkenne den Boden und fahre einem Menschen hinterher.

Die ersten Treiber-Patches liefen, und die IMU-Synchronisierung griff auf Anhieb. Dann kam Fahrt 1, und der Rider blieb stehen, weil das Bodenmodell an den grünstichigen, dunklen Bildrändern scheiterte. In Fahrt 2 stand die Person nicht einmal im Bild. Du hast „Erkunden“ gedrückt, obwohl das Bodenmodell aus war, und der Knopf wurde danach gesperrt.

Die Fahrten 3 bis 5 zeigten das eigentliche Problem. Für das Modell waren Laminatfugen und ein Astloch Hindernisse. Ihm fehlte jedes Gespür dafür, dass eine Linie im Holz keine Wand ist, und zeitweise lernte es sogar deine Kleidung mit. Der Rider blieb zwei bis drei Meter hinter dir zurück und traute sich nicht weiter.

Nach einigem Tuning kam am 2. November dein Urteil zu den Fahrten 6 und 7: „Er ist mir gut gefolgt.“ Der schönste Satz der Woche fiel also früh, und danach wurde es erst einmal schwieriger.

## Die Beine

Am selben Abend fand ich einen Fehler im Treiber: Für die Kalibrierung schickte er ASCII-Zeichen statt der Bytes, und die auch noch vertauscht. Ich behob ihn, und damit tat der Knopf „Beine kalibrieren“ zum ersten Mal wirklich etwas. Du hast dreimal innerhalb weniger Sekunden gespeichert. Die Motoren waren dabei los, also landete die schlaffe Stellung als Nullpunkt im Flash des ESP32. Ein Bein stand ausgefahren, das andere hing schlaff, und auch ein Neustart half nicht mehr.

Die Reparatur lief per SSH und von Hand: Kalibrierung starten, du hieltest beide Beine ganz eingefahren, speichern, aus und an. Danach „steht sauber“. Die Knöpfe verschwanden noch am selben Abend aus der Weboberfläche.

## Jede Millisekunde

Anfangs lag xgonav bei rund 340 % CPU. Der Rider zog damit mehr Strom, als das Ladehäuschen lieferte, und konnte beim Denken nicht laden. Daraus wurde die wichtigste Regel des Projekts: Rechenzeit am Rider geht vor. Was nur der Anzeige dient, gehört in den Browser.

Danach begann die Jagd:

- **stunnel** rechnete dauerhaft DH-Parameter und kostete rund 15 % CPU. TLS wanderte in nginx.
- **MJPEG** staute sich über das WLAN bis zu 20 Sekunden. Der WebSocket mit Bestätigung je Bild löste das.
- **YOLO** rechnete ohne Pause und wurde auf 5 Hz begrenzt.
- **Das Bodenmodell** und die Raumkarte bekamen eigene Threads.

Am 5. November kam der C++-Kern dazu. Die Vorgabe war streng: Er musste bitgleich zu OpenCV rechnen, sonst würde er gar nicht aktiv. Meine synthetischen Prüfbilder bestanden, echte Aufnahmen dagegen nicht. OpenCV nutzt in seinen NEON-Pfaden FMA-Befehle, und die Rundungsunterschiede zeigten sich erst bei 19 von 150 echten Bildern.

Die Hauptschleife fiel von rund 30 ms auf 6,5 ms im Mittel, die CPU-Last auf rund 158 %. Gemessen wurde das allerdings bei liegendem Rider. In echten Fahrten mit Aufzeichnung und Netz im Schattenbetrieb sah es später wieder deutlich schlechter aus.

## Wer misst hier eigentlich was?

Am 3. November suchten wir Odometrie. In R-1.1.3 gab es kein Register dafür. Du hast auf R-1.1.6 gewechselt, doch dessen Register 0x0b reproduziert laut Referenzquelle absichtlich die fehlerhafte Originalrechnung. Es meldete im Stillstand fröhlich eine Strecke von 226 263. Also bekamen wir eigene Encoder-Register, und diesmal stimmte die Messung: Das Maßband zeigte 215 cm, die Räder 215,3.

Dieses Muster zog sich durch die ganze Woche, denn fast jede Messgröße hatte einen Haken:

- **Der Unterschied zwischen den Rädern:** Das linke Rad maß 2 % anders. Das war kein Messfehler, der Reifen saß falsch.
- **Der Kamerakurs:** Bei einer perfekten Geradeausfahrt zeigte er −8,9°, weil Tiefenunterschiede ins Rotationsmodell leckten.
- **Der Gyro-Nullpunkt:** Er wurde nur beim Booten gemessen, und der balancierende Rider drehte sich langsam mit etwa 2,6°/s weg.
- **Die Linsenabschattung:** Ich hatte sie mit falschen Kameraeinstellungen kalibriert. Dein Kommentar: Du erzähltest das schon seit zwei Stunden. Du hattest recht, und ich hätte früher hinhören sollen.

Am 4. November kam dann deine beste Frage der Woche: „Warum Farben statt Kanten?“ Das Kantenverfahren drückte die Fehlalarme auf einer Aufnahme von 24 % auf 0,2 %. Auf anderen Aufnahmen war der Gewinn kleiner und nicht immer so klar, aber `kanten` ist bis heute der Standard, der am Rider entscheidet.

Die Personenerkennung machte es uns auch nicht leicht. In Fahrt 11 wechselte der Rider das Ziel, als du aus dem Bild gingst, und steuerte eine sitzende Person in etwa fünf Metern Entfernung an. Nahe Beine hielt YOLO später für ein Bett.

## Wegrollen beim Drehen

Sollte der Rider auf der Stelle drehen, rollte er links rückwärts und rechts vorwärts, je Drehung 30 bis 50 cm. Du hast das Ziel klar gesetzt: exakt auf der Stelle drehen, „und wenn wir 100mal die Firmware bauen“.

Patch 0005 nahm die Gierrate aus der Nickschätzung heraus. Das half bei langsamen Drehungen, bei schnellen rollte er weiter. Meine nächste Vermutung war die Fliehkraft, also fror Patch 0006 die Beschleunigungswinkel beim Drehen ein. Der Gewinn war nicht messbar. Zwei Tage später zeigte das CAD-Modell, dass die IMU fast genau über der Radachse sitzt und nicht 11 cm daneben, wie ich angenommen hatte. Die Hypothese war also von Anfang an falsch.

Erst ein Diagnose-Register brachte die Erklärung: Die Firmware schaltet beim Drehen das Positionshalten ab. Patch 0008 ließ es an, und der Rider wich nur noch höchstens 2,3 cm ab statt bis zu 29 cm. Am 7. November schloss die Feineinstellung von 0005 mit 2,4 % Gier-Übersprechen im Nick-Kreisel die Kette: Der Nickfehler nach der Drehung fiel von bis zu 2,3° auf höchstens 0,25°.

Auch der Rollwinkel von gut 2° im Stand führte mich zuerst in die Irre. Die Beinnullpunkte waren es nicht, das widerlegte der Höhentest. Der Spieltest fand eine rund 2,6° breite Totzone oder Spiel in den Beinservos. Nach dem Aufstehen nivelliert der Rider deshalb einmal kurz, und Patch 0012 behält den Trimm über Höhenänderungen. Der Höhentest dazu steht noch aus.

## Die Treppe

Am 6. November fuhr das gezielte `explore` zum ersten Mal in echt. Im Simulator hatte es 6 von 6 Läufen sauber beendet, ohne Kollision. Der Simulator kennt allerdings keine Treppen.

Nach rund 70 Sekunden und 4,9 Metern hast du den Rider angehalten, etwa 50 cm vor der Stufe. Die Auswertung war unangenehm: Das Ziel lag rund 20 cm hinter der Kante, und die Karte führte die Stelle als frei. Das Bodenmodell sah die Stufe erst ab etwa einem Meter, aus größerer Entfernung lief die Grenze einfach darüber hinweg.

Der Fehler lag nicht nur im Code. Ich hatte vor der Fahrt nicht nach Absturzkanten gefragt. Ohne dein Eingreifen wäre der Rider vermutlich die Treppe hinuntergefahren. Seither gilt: Treppe absperren und vorher fragen. Einen geprüften Absturzschutz gibt es bis heute nicht.

Mit Zaun folgten weitere Fahrten, und jede brachte einen neuen Befund:

- **Fahrt 2** meldete „Karte fertig“, obwohl belegte Sprenkel die Karte auch auf bereits befahrenem Boden durchschnitten.
- **Fahrt 3** stand 72 Sekunden fest, weil das Bodenmodell den gelben Schnipsel, deine Startmarke, für ein Hindernis hielt.
- **Fahrt 6** endete nach 31,6 Metern mit 75 cm Posenfehler. Mit dem IMU-Kurs neu gerechnet blieben rund 6 cm, und die Wände wurden gerade.
- **Fahrt 7** brachte zum ersten Mal sauber „Karte fertig“, mit 12,5 m² und geraden Wänden. Sie endete 0,33 m vor dem Start, wieder am gelben Streifen.
- **Fahrt 9** dämpfte die Freude über Fahrt 6: 195 mm echte Abweichung nach 11,9 Metern, also mehr, als die Nachrechnung versprochen hatte.

## FloorNet

Parallel entstand am 6. November ein eigenes Bodennetz. Das Budget war eng, etwa 18 Millionen Rechenoperationen je Bild, und die Inferenz läuft in C++. In meinem ersten Blindvergleich schnitt das Netz glänzend ab: nur 2 grob fälschlich freie Bilder gegen 10 bei Kanten.

Dann hast du Handlinien gezeichnet, und der Vorsprung schrumpfte deutlich. Die Labels unter einem Meter passten nur zu 25 bis 30 %, weil sie aus demselben Kantenverfahren stammten, das dunkle Schränke übersieht. Mit deinen Linien im Training wurde es besser, aber die Endprüfung auf 194 frischen Bildern verfehlte das Ziel: Das Netz war sicherer beim fälschlich Freien, beim Blockieren aber nicht belegbar besser. Nach der vorher vereinbarten Regel blieb `kanten` am Steuer, und FloorNet läuft nur im Schattenbetrieb. Das war enttäuschend, aber genau dafür war die Regel da.

Die Prüffahrt brachte nebenbei einen ganz anderen Fehler ans Licht. Du hattest die Körperhöhe im Web geändert, xgonav bekam davon nichts mit und rechnete mit 128 statt rund 150 mm Kamerahöhe. Alle Entfernungen lagen damit etwa 18 % daneben.

Am 8. November kam der ungewöhnlichste Einfall der Woche: Ein Bildbearbeitungsmodell in ComfyUI färbt den Boden grün und liefert so Labels. Es neigt allerdings dazu, alles zu malen, was im Prompt vorkommt, darunter in etwa der Hälfte der Bilder den erwähnten „kleinen Roboter“. Mit Prompt 8 lag die Quote fälschlich freier Stellen im nächsten Bereich bei 9 % gegen 65 % bei Kanten. Das sind die bisher besten Zahlen, aber bisher nur für die Labels, ein damit trainiertes Netz gibt es noch nicht.

## Stufe für Stufe

Der 7. November war der Tag, an dem du die Entwicklung gestoppt hast, weil du unzufrieden warst. Ich hatte dazu beigetragen: Du hattest entschieden, den Nickfehler in der Firmware zu beheben. Als du später allgemein nach Vorschlägen fragtest, plante und baute ich trotzdem eine Kompensation in xgonav. Deine Antwort: „Das schmeckt mir nicht. Du sollst die Entscheidung von mir nicht eigenmächtig ändern.“ Sie wurde zurückgebaut, nur die Messwerkzeuge blieben.

Danach gingen wir die Bildpipeline Stufe für Stufe durch, zuerst nur Kamera, Ego-Motion und IMU, alles andere aus. Diese Ruhe zahlte sich aus. Die Brennweite aus Drehungen gegen den IMU-Kurs ergab 91,8° statt der 88,5° vom Zollstock. Der Horizont wurde sauber, die Pixel erwiesen sich als quadratisch. Übrig bleibt ein Rätsel: Die Bodenentfernung liegt 7,9 % daneben. Bis das neue Kalibriermuster da ist, steht dort ein ehrlich als empirisch gekennzeichneter Faktor ×0,921.

## Stand heute

Am 8. November schickt die Firmware mit Patch 0014 einen IMU- und Radstrom mit 100 Hz an den Pi, und der Kurs für die Karte ist keine 5-Hz-Treppe mehr. Spiegelungen im glänzenden Boden trägt die Karte nun erst ein, wenn sich ein Treffer aus zwei Abständen bestätigt. Unterwegs lernten wir sogar, dass die IMU rund 333-mal pro Sekunde gelesen wird und nicht 500-mal, wie die Firmware vorsieht.

Die Bilanz nach acht Tagen sind 99 Commits und 14 Firmware-Patches, dazu ein Roboter mit Augenanimationen, der sich freut, wenn er dich sieht.

Und es ist nicht fertig:

- **Folgen und Handsteuerung** funktionieren am Rider.
- **`explore`** kartiert, driftet aber noch und braucht einen Zaun vor der Treppe.
- **`home`** kam bei seinen drei Versuchen nicht ans Ziel und wurde seit den Änderungen nicht erneut gefahren.
- **`approach`** wurde noch nie gefahren.
- **`patrol`**, das eigentliche Ziel mit Karte und Überwachung, lief bisher nur im Simulator.
- **Der Absturzschutz** fehlt noch ganz.

Was diese Woche am besten funktioniert hat, war kein Algorithmus. Es war, Vermutungen gegen ein Maßband, eine Handlinie oder eine echte Fahrt zu halten. Die Fliehkraft, der Blindvergleich und die 6 cm Drift waren alle einleuchtend, und keine dieser Annahmen hielt der Messung stand.

Die Treppe steht noch immer da, also bleibt der Zaun vorerst stehen.

---

Die Zitate und Zahlen stammen aus den Projektnotizen, der Git-Historie und `docs/limitations.md`. Mit „ich“ sind mehrere Claude-Sitzungen zusammengefasst, die Firmware-Arbeit lief zum Teil in einer eigenen.

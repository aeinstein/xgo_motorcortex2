# Ideen für einen lebensechteren Modus pet

Stand 2026-11-28. Nutzerwunsch: Erweiterungen, die `pet` lebensechter machen, auch gewagte; Voraussetzungen
aufschreiben; mehr Töne und Animationen; Augengenerator; Akkustand im Verhalten. ✦ = gewagt. In Klammern die
Kosten am Pi (Rechenzeit hat Vorrang, CLAUDE.md).

**Umgesetzt:** Augengenerator im Display-Daemon (2026-11-28, `docs/display-emotions.md`, am Rider eingespielt und
liegend geprüft). Alles andere ist offen.

## Bestand (2026-11-28)

- pet ist Ereignis → Reaktion mit Abklingzeiten. Innere Größen nur `trust` (global, nicht je Person), `fright`,
  `habit`; Langeweile = Zeit seit der letzten Begegnung. Gedächtnis nur im RAM; jeder Moduswechsel baut ein neues
  `Pet`.
- Töne: `tools/sounds.py` synthetisiert mit numpy, 15 pet-Töne × 3 Tonhöhen als WAV, ein `aplay`-Prozess je Ton.
- Ungenutzt: Nutzer-LEDs (`/proc/XGORider/leds/<i>`), freie Tasten, Firmware-Aktionen, Hebeschutz-Zähler, Nicken,
  die Emotionen `Angry`/`hate`.

## Leitidee: Hirnstamm und Großhirn

Lebensecht wirkt ein Tier weniger durch mehr Auslöser als durch:
- **nie ganz starr** (Lidschlag, Sakkaden, Atmen)
- **Stimmung färbt alles**
- **Gedächtnis**
- **keine exakte Wiederholung**
- **Timing**: erst schauen, dann drehen; ausholen, nachschwingen

Aufteilung nach Rechenzeit:
- **Hirnstamm**: kleine C-Teile, die selbständig weiterleben (Augen im Display-Daemon, Stimme). Sie bekommen nur
  Ziele und Stimmung, nie Einzelbilder.
- **Großhirn**: `pet.py`. Es entscheidet selten (Regelschritt oder 1 Hz) über ein paar Floats.

## A. Augen (Augengenerator vorhanden)

1. ~~Lebendiger Ruheblick~~ (umgesetzt): Sakkaden, Mikrosakkaden, Lidschlag nach Zufall, Doppelblinzler, Atmen.
2. ~~Vergenz~~ (umgesetzt): Bei nahem Ziel rücken die Augen zusammen. Offen: ganz nah leicht schielen und verlegen
   werden.
3. **Pupille bzw. Größe nach Licht und Erregung**: Die Helligkeit misst `dunkel.py` schon (4 Hz). Bei Katze oder
   Interesse weit, bei Angst klein. (0)
4. **Augen mit Trägheit**: Beim Bremsen, Anfahren und Drehen schwappen die Augen. Gierrate und Beschleunigung
   kommen aus `ImuFrame`, ein paar Byte je Regelschritt an den Daemon, die Feder rechnet dort. (≈ 0)
5. **Erst schauen, dann drehen**: Der Blick führt jede Drehung 150–300 ms vor `dreh_w`; vor dem Anfahren kurz
   ausholen. Teil 1, der Blick springt sofort, ist umgesetzt. (0)
6. **Symbole**: `?` (doubt, Horchen ohne Ergebnis), `!` (Schreck), kleines ♥, `Zzz`, ♪ (Dudeln, Tanzen),
   Schweißtropfen (festgefahren, Anschub an Fuge), Wangenröte (shy/like bei Bekannten), Akku (Hunger). Als
   Abstandsfeld wie das Herz in `augen.c`. (Daemon)
7. **Stimmungsfarbe**: leichte Tönung nach Stimmung, Personenfarbe stufenlos (`farbe RRGGBB`). (0)
8. **Augen-Stile** ✦: Katzenaugen (Schlitzpupille, im Dunkeln rund), Eule, Visier; im Web wählbar. (0)
9. **Träumen** ✦: Im Ruhemodus zucken die Augen selten unter geschlossenen Lidern (REM). Ab und zu erscheint eine
   Traumblase mit dem, was er heute erlebt hat (Katze, Ball, Person), aus dem Gedächtnis. (Daemon, ~1 Bild/s in
   Schüben)
10. **Live-Augen und Augen-Editor im Web**: Der Browser zeichnet dieselben Parameter (JS-Port des Modells), die
    Werte kommen als wenige Byte über den Live-WebSocket. Ein Editor mit Reglern schreibt neue Ausdrücke. (0 am Pi)

## B. Stimme (Stimmgenerator in C)

1. **Babbelsprache**: Silben aus Vokal-Formanten (a/e/i/o/u, zwei Formantfilter) mit Tonhöhenkontur, Vibrato und
   Rauheit. Jede Äußerung wird neu gewürfelt, keine klingt zweimal gleich. (< 1 ms je Äußerung)
2. **Prosodie aus Stimmung**:

   | Stimmung | Stimme |
   |---|---|
   | froh | hoch, steigend, schnell |
   | traurig | tief, fallend, langsam |
   | Angst | zitternd |
   | müde | gedehnt |
   | Frage (doubt) | endet steigend |
3. **Tonfall-Echo** ✦: Den Grundtonverlauf einer gehörten Äußerung rechnet `stimmfast` schon. Er antwortet in
   eigener Babbelsprache mit derselben Melodie. (≈ 0 zusätzlich)
4. **Augen sprechen mit**: Die Augen pulsieren mit der Hüllkurve der eigenen Stimme; der Daemon bekommt Start und
   Silbenrhythmus. (0)
5. **Neue Laute**: Gähnen, Niesen nach dem Aufwachen, leises Schnarchen beim Dösen, Schmatzen, Summen beim Warten.
   Dazu Schluckauf ✦: 3–5 Hickser, bei denen Körper und Augen zucken.
6. **Dauerhafter Ausgang**: Der ALSA-Ausgang bleibt in einem Thread offen, statt je Ton einen `aplay`-Prozess zu
   starten. Das spart fork/exec, erlaubt Überblenden und Abbrechen und macht `klingt()` exakt. (weniger als heute)

## C. Innenleben

1. **Triebe** (0..1, mit 1 Hz fortgeschrieben, ein paar Floats je Sekunde):
   - Energie: Akku, Tageszeit, Aktivität seit der letzten Ruhe
   - Geselligkeit: steigt ohne Kontakt
   - Neugier: steigt bei Gleichförmigkeit, sinkt durch Neues
   - Spieltrieb
   - Erregung: Schreck, Lärm; klingt ab
2. **Stimmung** = Valenz × Erregung, ändert sich langsam (Minuten). Sie färbt:
   - Augen-Grundhaltung und Stimme
   - Fahrtempo-Faktor (≤ 1)
   - Grundhöhe (± wenige mm, nie unter Minimum + 10 mm)
   - Schreckschwellen
3. **Tätigkeitswahl nach Nutzwert**: schlafen, Menschen suchen, streifen, spielen, betteln (Energie niedrig und
   eine Person da), Lieblingsplatz, sich schütteln oder putzen. Ersetzt die festen Schwellen
   `roam_after_s`/`bored_s`/`doze_s` durch Triebschwellen mit Hysterese und Zufall.
4. **Persönlichkeit** (fest, in TOML oder im Web): Mut, Verspieltheit, Anhänglichkeit, Gesprächigkeit wirken als
   Gewichte. ✦ Langsame Drift über Wochen: Er wird mutiger, wenn Schrecke folgenlos bleiben.
5. **Dunkelangst ankoppeln**: weniger Angst, wenn eine vertraute Person da ist.
6. **Energie** = Akku (Block H) × Müdigkeit aus Aktivität (gefahrene Meter + Drehungen der letzten 30 min) ×
   Tageszeit; der Akku setzt die Obergrenze.

## H. Akku im Verhalten (Nutzerwunsch 2026-11-28)

**Grundlage:**
- `/proc/XGORider/battery` liefert nur Prozent, etwa alle 2 s.
- Unter 8 % schaltet der Treiber den Pi ab.
- Es gibt keine Ladeerkennung.
- Volle Bildverarbeitung zieht mehr, als die Ladestation liefert.

**Heute** gibt es in pet nur Sperren:
- < 40 % (`roam_battery`): kein Streifen
- ≤ 20 % (`battery_low`): keine Ohnmacht, kein Aufstehen
- dazu `shy` alle 300 s bei ≤ 20 % (`EmotionTrigger`)

1. **Akku = Hunger**: Der Wert wird geglättet (Median über 60 s, Hysterese 3 Punkte). Beim Fahren sackt er ab;
   ungeglättet würde die Stimmung flackern.
2. **Stufen** (Grenzen in `[pet]`, wirken als Faktoren auf vorhandene Werte):

   | Akku | Stimmung | Verhalten |
   |---|---|---|
   | ≥ 60 % | munter | volles Repertoire, spielt, streift, tanzt |
   | 40–60 % | gesetzt | kürzere Etappen, langsamere Gesten, seltener Dudeln |
   | 25–40 % | hungrig | kein Streifen (wie heute), schwerere Lider, ab und zu Akku-Symbol im Auge + Magenknurren ✦; sucht Gesellschaft, bettelt bei einer Person (grievance, wimmern) |
   | 15–25 % | erschöpft | fährt nur noch zum Startpunkt („Körbchen“, home mit Startbild) und setzt sich ab; Bildverarbeitung aus, nur die Ohren bleiben an; eine Stimme weckt ihn kurz mit müden Augen, er steht nicht auf |
   | < 10 % | „Gute Nacht“ | sleep, leiser trauriger Ton, LEDs dunkel, bevor der Treiber bei 8 % abschaltet |

3. **Energiekosten in der Tätigkeitswahl**: Fahren kostet mehr als Schauen, Schauen mehr als Brabbeln. Je leerer
   der Akku, desto mehr billige Ausdrucksmittel (Augen, Ton, Kopf schief) statt Fahrten.
4. **Rechenzeit ist Strom**: Bei niedrigem Akku läuft der Detektor nur mit `doze_hz`, die Gesichtserkennung ist
   aus. FloorNet bleibt unangetastet (einzige Bodengrenze).
5. **Laden erkennen**:
   - Indirekt: Steigt der Wert im Stand binnen 10 min um ≥ 2 Punkte, „frisst“ er (gurren, like).
   - Dann legt er sich schlafen (Ruhemodus mit Bildverarbeitung aus, sonst lädt er nicht).
   - Bei ≥ 90 % wacht er auf: Strecken, Gähnen, munter.
   - Sauberer wäre ein Ladeflag samt Spannung aus der Firmware (siehe Voraussetzungen).
6. **Anzeige**: Lider, Blinkrate (`rate`) und LED-Helligkeit folgen der Energie; im Auge ein Akku-Symbol statt
   `shy`.

Kosten am Pi: ein Median über 30 Werte alle 2 s, ein paar Vergleiche je Regelschritt.

## D. Gedächtnis

Datei `/var/lib/xgonav/gedaechtnis.json`, höchstens einmal je Minute geschrieben.

1. **Personen**: Vertrauen je Person, letzte Begegnung, Häufigkeit. Die Wiedersehensfreude wächst mit der
   Abwesenheit: nach 5 min kurz, nach 8 h ein Freudenausbruch. Bei Fremden ist er vorsichtiger.
2. **Orte** (braucht eine bestätigte Pose in einer gespeicherten Karte):
   - Lieblingsplatz und Schlafplatz
   - gemiedene Orte: Schreck, Sturz, Hund; die Dunkelzone dauerhaft
   - die Stelle, an der Menschen meist auftauchen („Tür“)
3. **Tageslauf** ✦: Er erwartet Personen zu ihren üblichen Zeiten und wartet an der „Tür“.
4. **Der Zustand übersteht Moduswechsel und Neustart.**

## E. Körpersprache

1. **Atmen im Stand**: Höhe ±2–3 mm bei 0,2–0,3 Hz, die Rate richtet sich nach der Erregung. Die Geometrie nimmt
   `body_height` aus dem Strom, FloorNet bleibt unberührt. (1 Schreiben je 0,5 s)
2. **Gewichtsverlagerung**: langsames Roll-Pendeln um ±1–2° im Ruhen.
3. **Strecken nach dem Aufwachen**: ganz hoch, Roll links und rechts, Gähnen, die Augen machen mit.
4. **Schütteln** ✦ nach Schreck oder Absetzen: schnelles Roll-Wackeln wie ein nasser Hund (Roll-Rate der Firmware
   prüfen), oder als Firmware-Aktion ab Nr. 7.
5. **Freudenhüpfen** ✦: schneller Höhenimpuls runter und hoch bei großer Wiedersehensfreude. Ein echter Sprung nur,
   wenn Firmware und Mechanik das hergeben (eigenes Firmware-Thema).
6. **LEDs**:
   - Nutzer-LEDs in Stimmungsfarbe, Puls bei like, Glimmen beim Dösen.
   - Animationen als Firmware-Effekt (Firmware-Sitzung), damit der Pi nicht 20-mal je Sekunde schreibt; der Pi
     schreibt nur bei Ereignissen.
   - Beim Fahren zeigt die Firmware grün; Ausdruck also nur im Stand.

## F. Wahrnehmung und Spiele

1. **Augen zuhalten** ✦: Das Bild wird binnen < 0,3 s von hell fast schwarz, der Rider steht. Er ist verwirrt;
   ist die Hand weg, lacht er. Abgrenzung zur Dunkelangst: dort 2 s, und vorher ist es schon dunkel. (Die
   Helligkeit gibt es schon.)
2. **Klaps oder Streicheln per IMU**: Eine kurze Spitze im Stand ohne Fahrbefehl ist ein Klaps auf den Kopf: Er
   freut sich, bei einem harten Klaps ist er beleidigt. Erkannt in Firmware oder Treiber (Ereignis im Strom),
   nicht in Python je Probe.
3. **Hochheben**: Die Firmware erkennt das Abheben (Hebeschutz, Reg 0x19, Zähler `trips` in
   `/proc/XGORider/lift`). Dann „Wiii!“ und große Augen, beim Absetzen „puh“. Nur Ausdruck.
4. **Lob und Tadel an der Prosodie** ✦: Eine hohe, singende Stimme ist Lob (like, Vertrauen +), tief, laut und
   kurz ist Tadel (shy, duckt sich). Grundton und Pegel liegen vor. (wenige Vergleiche)
5. **Namen lernen** ✦: Eine Stimmprobe wird im Web als „Name“ markiert. Erkennt er sie, dreht er sich um und
   kommt (`stimme.py`, MFCC+DTW gegen eine Probe).
6. **Pfeifen und Klatschen**: Ein stabiler hoher Grundton > 0,5 s heißt: Er kommt wie ein Hund. Zweimal Klatschen
   heißt: komm.
7. **Musik und Tanzen** ✦: Den Takt liefert das Mikrofon (Onset-Stärke + Autokorrelation, in C am Ortungsstrom).
   Er wippt mit Höhe und Roll im Takt, die Augen hüpfen, dazu ♪. (~0,5–1 % eines Kerns, nur in pet bei erkannter
   Musik)
8. **Ball und Kuscheltier** ✦: Die YOLO-Klassen `sports ball` und `teddy bear` stehen in COCO, sind in `[detector]`
   nur nicht freigeschaltet (gleiches Netz, keine Mehrkosten). Als Ziel verfolgen bis davor. Anstupsen widerspräche
   „Erkennungen nie Hindernis, der Boden entscheidet“: nur als ausdrückliche Abweichung, wenn der Nutzer das will.
9. **Blickkontakt und gemeinsame Aufmerksamkeit** ✦: Aus den 5 YuNet-Landmarken grob die Kopfdrehung schätzen.
   Schaut die Person ihn an, hält er den Blick; schaut sie weg, schaut er in dieselbe Richtung. (YuNet läuft schon
   mit ≤ 2 Hz.)
10. **Spiegel** ✦: Er erkennt die eigenen Augen im Bild, wird neugierig, dann shy. (selten prüfen)
11. **Laserpunkt jagen** ✦: ein kleiner, sehr heller roter Fleck am Boden (Schwelle auf 80 × 60 in C). Er jagt ihm
    hinterher wie eine Katze. (~0,1 ms je Bild, nur in pet)
12. **Kraulen per Taste**: Eine freie Taste (`/proc/XGORider/buttons`, alle 200 ms abgefragt) heißt gekrault. Die
    Augen werden zu schmalen glücklichen Bögen, er schnurrt, Vertrauen +. (eine Datei je Regelschritt lesen, nur
    in pet)
13. **Ankunft per WLAN** ✦: Das Handy des Nutzers erscheint in der Nachbartabelle (alle 30 s lesen, MAC in der
    Konfig). „Herrchen kommt“: Er wartet an der „Tür“. (eine Datei alle 30 s)

## G. Noch gewagter

1. **Körbchen** ✦: Bei niedrigem Akku fährt er zum Körbchen (home + Startbild gibt es), legt sich hin und schläft.
   Selbst laden braucht Hardware (Kontaktplatte oder Pogo-Pins, Ladeerkennung). Ohne sie zeigt er Hunger, kommt zum
   Menschen und bettelt.
2. **Echtes Streicheln** ✦: Kupferfolie auf dem Kopf an einem freien Touch-Pin des ESP32 (Hardware + Firmware).
3. **Tricks lernen** ✦: Ein Wort (Stimmprobe) fällt, er probiert eine zufällige Geste. Lob verstärkt die
   Verknüpfung Wort → Geste (Tabelle im Gedächtnis), über Tage.
4. **Eifersucht** ✦: Beschäftigt sich eine Person mit der Katze, drängelt er sich dazwischen (grievance).
5. **Tagebuch** ✦: die Ereignisliste des Tages als Kachel im Web (am Pi nur die Liste).

## Voraussetzungen

| Voraussetzung | für | Art | Stand |
|---|---|---|---|
| Augengenerator im Display-Daemon | A, Ausdruck überall | C, Daemon | 2026-11-28 am Rider, liegend geprüft |
| Stimmgenerator + dauerhafter ALSA-Ausgang | B | C | neu |
| Pet-Zustand dauerhaft (Gedächtnisdatei) | C, D | Python, selten | neu |
| Bestätigte Pose über Sitzungen | D2, D3, G1 | Karte | laufender Abgleich falsch, Startbild-Abgleich ok |
| Klopf-Ereignis bzw. Beschleunigung im IMU-Strom | F2 | Firmware/Treiber | prüfen |
| LED-Effekte in der Firmware | E6 | Firmware-Patch | Firmware-Sitzung |
| Kleine Höhen- und Roll-Sollwertschritte ruckfrei | E1–E5 | am Rider prüfen | offen |
| Beat-Tracker | F7 | C am Ortungsstrom | neu |
| Ladeflag + Spannung im Strom (Firmware kennt die Spannung, zeigt sie nur in der BLE-Konsole `!status`) | H5, G1 | Firmware + Treiber | prüfen, ob die Hardware Laden erkennt |
| Hebeschutz-Zähler `trips` regelmäßig (jedes Lesen von `/proc/XGORider/lift` fragt den ESP32 synchron, bis 100 ms) | F3 | Treiber: besser als Ereignis im Strom | heute nur nach Sturz gelesen |
| Freie Tasten (A–D; belegt nur A+B, C+D, B) | F12 | vorhanden | ungenutzt |
| Neue Firmware-Aktionen ab Nr. 7 (Schwanzwedeln, Tanz, Schütteln) | E4, E5, F7 | Firmware-Patch | Tabelle `kActions`, frei 7–230 |
| Ladeplatte + Ladeerkennung | G1 | Hardware | – |
| Freier Touch-Pin am ESP32 | G2 | Hardware + Firmware | prüfen |
| Entscheidung Ball anstupsen | F8 | Nutzer | offen |

## Vorgeschlagene Reihenfolge

1. Augengenerator: am Rider. Offen ist das Aufräumen (nur mit Zustimmung des Nutzers): −1 GB SD und −2,5 min
   Einspielen.
2. Innenleben mit Akku (C, H) und Gedächtnis (D), damit Energie und Stimmung die Augen färben.
3. Stimmgenerator (B).
4. Kleine Körpersprache (E1, E2, A5), am Rider mit Beobachtung.
5. Spiele nach Wahl des Nutzers (F).

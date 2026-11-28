---
name: geschichte
description: Die Projektgeschichte web/geschichte/ („Zwei Wochen auf zwei Rädern“, ein Kapitel je Seite) fortschreiben – neue Kapitel seit dem letzten Stand in trocken-sarkastischem, lustigem Ton, Stil vorher am bestehenden Text prüfen, Rechtschreibung korrigieren. Verwenden, wenn der Nutzer die Geschichte weiterschreiben, ergänzen oder auf den neuen Stand bringen will.
---

# geschichte: Projektgeschichte fortschreiben

Ordner `web/geschichte/` (statisch, kein PHP; nginx `location /geschichte/`, `/geschichte.html` leitet per 301 um):
- `index.html`: Titel, Untertitel, Inhalt als `<ol class="inhalt">` (je Eintrag Link + `<span class="datum">`), Fußnote.
- `NN-slug.html`: ein Kapitel je Seite (seit 2026-11-28). Slug aus der Überschrift (ä→ae, ö→oe, ü→ue, ß→ss,
  Kleinbuchstaben, Bindestriche). Aufbau: `<p class="kopf">` (Link zum Inhalt · Kapitel N · Datum), `<h1>`, Text,
  `<nav class="blaettern">` (zurück · Inhalt · weiter; erste Seite ohne zurück, letzte ohne weiter).
- Letzte Seite ist immer `NN-stand-heute.html`, mit der Fußnote (`<hr>` + Quellen) vor der Blätterleiste.
- `stil.css`: gemeinsames CSS, hell und dunkel. Kein neues CSS.

Daneben steht `web/tagebuch/index.html` (Tag für Tag, sachlicher). Die Geschichte erzählt, das Tagebuch protokolliert;
beide nicht vermischen.

## Ablauf

1. **Stil prüfen, bevor geschrieben wird.** Alle Kapitel lesen (nicht nur das Ende) und kurz festhalten:
   Erzählperspektive, Ton, Satzlänge, wie Pointen sitzen, wie Zitate eingebaut sind, letztes erzähltes Datum, Zahlen
   in „Stand heute“. Weichen die letzten Kapitel vom Rest ab (zu technisch, zu lang, Pointe fehlt), das dem Nutzer
   melden, bevor neue Kapitel dazukommen.
2. **Rechtschreibung prüfen**, auch in den Zitaten. Nutzerwunsch 2026-11-28: Zitate des Nutzers bekommen korrekte
   Rechtschreibung, Groß- und Kleinschreibung und Zeichensetzung, Tippfehler werden korrigiert („vorraus“ →
   „voraus“), weil er am Handy oft falsch abschickt. Der Wortlaut bleibt, ebenso Umgangssprache („nix“, „’ne“,
   „scheiße“) und Großschreibung als Betonung („IMMER“). Erzähltext auch auf Bezugsfehler prüfen (Genus: „das Modell
   … für es“). Die Korrekturen in der Antwort auflisten.
3. **Stoff seit dem letzten Stand sammeln** (Datum aus Schritt 1). Quellen, in dieser Reihenfolge:
   - `git log --since=<Datum> --stat` und Commit-Nachrichten,
   - `web/tagebuch/index.html` (falls schon nachgetragen),
   - Projektnotizen: `MEMORY.md` samt verlinkten Dateien im Memory-Ordner, `TODO.md`, `docs/limitations.md`,
   - Sitzungsverläufe (`~/.claude/projects/G--ALLPROJECTS-PycharmProjects-xgo-motorcortex2*/*.jsonl`, auch `-web`
     und `-firmware`, früher `C:\rider`) für **wörtliche Nutzerzitate** und den tatsächlichen Hergang. Zitate nur
     übernehmen, wenn sie dort so stehen, und nie erfinden. Korrigiert wird nur die Rechtschreibung (Schritt 2).
   Die Durchsicht der Verläufe an Explore-Agenten geben, mit Zeitraum und der Bitte um wörtliche Zitate samt Uhrzeit.
4. **Gliedern.** Neue Kapitel nach Themen, nicht nach Tagen (Tage gehören ins Tagebuch). Jedes Kapitel hat einen
   Bogen: Vorhaben → was schiefging → Befund (meist: der Nutzer hatte recht oder die Messung widersprach) → Stand.
   Jedes neue Kapitel ist eine neue Datei vor „Stand heute“. Danach „Stand heute“ umnummerieren, den Eintrag im Inhalt
   ergänzen und die Blätterlinks der Nachbarn anpassen (vorheriges Kapitel → neues, neues → „Stand heute“). Gibt es
   viele neue Kapitel, die Seiten mit einem kleinen Skript im Scratchpad erzeugen, nicht von Hand.
5. **„Stand heute“ und Schluss neu schreiben**, nicht anhängen: Datum, Anzahl Commits (`git rev-list --count HEAD`),
   Firmware-Patches (`firmware/patches/`), Statusliste je Modus (gefahren / eingebaut, nicht gefahren / nur
   Simulator). Keinen Absturz- oder Treppenhinweis in den Stand schreiben (Nutzer 2026-11-20). Der Schlussabsatz
   greift ein Leitmotiv auf (Maßband, Python gegen C++, „nie gefahren“) und endet mit einer trockenen Pointe. Die
   Fußnote bleibt, Quellen ggf. ergänzen. Den Titel („Zwei Wochen …“ in `index.html`, im `<title>` und in jeder
   `.kopf`-Zeile) bei längerem Zeitraum mit dem Nutzer abstimmen, nicht eigenmächtig ändern.
6. **Endkontrolle:** Rechtschreibung der neuen Abschnitte, jede Zahl gegen die Quelle. Im Text nur `<p>`, `<ul><li>`,
   `<strong>`, `<code>`, `<em>`, Zitate mit „…“ statt `<q>`. Alle Links zeigen auf vorhandene Dateien, der Inhalt
   listet jede Seite, die Dateien sind UTF-8 mit LF.
7. **Immer sofort auf den Rider einspielen** (Nutzerwunsch 2026-11-18, ohne Rückfrage): Rider-Adresse
   192.168.53.110 oder 10.10.0.110 probieren, den Ordner per `scp -r` nach `/opt/xgonav/web/geschichte/` kopieren
   (vorher dort löschen, damit umbenannte Kapitel nicht liegen bleiben), dann `chown -R aeinstein:aeinstein`, md5
   gegen lokal prüfen und `curl -sk https://localhost/geschichte/` → 200. Keine Sicherung („Sicherung“ = Commit),
   kein Neustart. Dem Nutzer die neuen Kapitel kurz nennen. **Commit nur auf Zuruf** (auf dem aktuellen Branch, nur
   die eigenen Dateien, Präfix „Geschichte: …“, keine Session-Zeile).

## Stil (aus dem bestehenden Text)

- **Perspektive:** „ich“ = Claude (alle Sitzungen zusammengefasst, auch der Firmware-Claude), „du“ = der Nutzer,
  „wir“ für Gemeinsames, „der Rider“/„er“ für den Roboter. Der Nutzer wird nie mit Namen genannt, auch nicht über
  Namen aus der Gesichtserkennung.
- **Ton:** trocken, selbstironisch, sarkastisch, aber nie gehässig. Der Spott trifft vor allem Claude (baut zuerst in
  Python, ist von C++ überrascht, glaubt eigenen Kennzahlen mehr als dem Bild), dann die Technik (Firmware, die
  selbst nicht weiß, was sie tut; Simulator, der Wände erfindet), den Nutzer höchstens liebevoll.
- **Pointen am Absatzende**, kurz und lakonisch, oft als eigener Satz:
  „Synthetische Bilder lügen nicht, sie sagen nur nichts.“ –
  „Ein Knopf, der den Flash beschreibt, ist kein Knopf, sondern eine Falle mit Beschriftung.“ –
  „Er hatte sich also vor allem umgesehen.“
- **Vergleiche** aus dem Alltag, sparsam (höchstens einer je Absatz): „wie ein Hund, der dem Parkett nicht traut“,
  „wie ein Auto mit Schluckauf“, „wie jemand, der sehr gern möchte“.
- **Fakten exakt:** Datum, Uhrzeit, Messwerte in deutscher Schreibweise (0,5 ms; 21 866; 2,3 cm; −31°).
  „Übertrieben ist der Ton, nicht die Zahlen.“ Nichts beschönigen: Fehler von Claude offen benennen („Es war
  meine“), Ungefahrenes als ungefahren kennzeichnen.
- **Zitate:** in „…“, mit korrigierter Rechtschreibung (Schritt 2). Ein Zitat pro Gedanke, gern als Wendepunkt („Das
  Netz hatte recht“).
- **Sätze:** mittellang, einfache Hauptsätze, kein Fachjargon ohne Erklärung, Code-Namen in `<code>`. Keine Emojis,
  keine Ausrufezeichen, keine Superlative im Erzählton, keine Gedankenstrich-Ketten.
- **Leitmotive** wieder aufnehmen: Maßband und Zollstock gegen Vermutung, Python gegen C++, Rechenzeit am Rider,
  „nie gefahren“, der Nutzer als Bote zwischen mehreren Claudes.
- **Deutsch mit korrekten Umlauten und ß**, auch im HTML (Dateien sind UTF-8). Lagewinkel heißen Nick, Roll, Gier,
  nie „Neigung“.

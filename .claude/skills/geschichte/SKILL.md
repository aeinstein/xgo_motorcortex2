---
name: geschichte
description: Die Projektgeschichte web/geschichte.html („Zwei Wochen auf zwei Rädern“) fortschreiben – neue Kapitel seit dem letzten Stand in trocken-sarkastischem, lustigem Ton, Stil vorher am bestehenden Text prüfen, Rechtschreibung korrigieren. Verwenden, wenn der Nutzer die Geschichte weiterschreiben, ergänzen oder auf den neuen Stand bringen will.
---

# geschichte: Projektgeschichte fortschreiben

Datei: `web/geschichte.html` (eine statische Seite, kein PHP). Daneben `web/tagebuch/index.html` (Tag für Tag, sachlicher).
Die Geschichte erzählt, das Tagebuch protokolliert; beide nicht vermischen.

## Ablauf

1. **Stil prüfen, bevor geschrieben wird.** Die ganze `geschichte.html` lesen (nicht nur das Ende) und kurz
   festhalten: Erzählperspektive, Ton, Satzlänge, wie Pointen sitzen, wie Zitate eingebaut sind, letztes
   erzähltes Datum, Zahlen in „Stand heute“. Abweichungen der letzten Kapitel vom Rest (zu technisch, zu lang,
   Pointe fehlt) dem Nutzer melden, bevor neue Kapitel dazukommen.
2. **Rechtschreibung im Bestand prüfen.** Erzähltext auf Rechtschreib-, Grammatik- und Bezugsfehler durchsehen
   (z. B. Genus: „das Modell … für es“, nicht „für ihn“) und korrigieren. **Zitate des Nutzers bleiben
   wörtlich**, samt Tippfehlern („eigenmöchtig“, „vorraus“, „bervorzugen“): Die Fußnote verspricht, dass die
   Zitate echt sind. Gefundene Korrekturen in der Antwort auflisten.
3. **Stoff seit dem letzten Stand sammeln** (Datum aus Schritt 1). Quellen, in dieser Reihenfolge:
   - `git log --since=<Datum> --stat` und Commit-Nachrichten,
   - `web/tagebuch/index.html` (falls schon nachgetragen),
   - Projektnotizen: `MEMORY.md` samt verlinkten Dateien im Memory-Ordner, `TODO.md`, `docs/limitations.md`,
   - Sitzungsverläufe (`~/.claude/projects/G--ALLPROJECTS-PycharmProjects-xgo-motorcortex2/*.jsonl`, auch
     `C:\rider`) für **wörtliche Nutzerzitate** und den tatsächlichen Hergang. Zitate nur übernehmen, wenn sie
     dort so stehen; nie erfinden oder glätten.
   Breite Durchsicht der Verläufe an einen Explore-Agenten geben, mit Zeitraum und der Bitte um wörtliche Zitate
   samt Uhrzeit.
4. **Gliedern.** Neue `<h2>`-Kapitel nach Themen, nicht nach Tagen (das ist das Tagebuch). Je Kapitel ein Bogen:
   Vorhaben → was schiefging → Befund (meist: der Nutzer hatte recht / die Messung widersprach) → Stand.
   Kapitel vor „Stand heute“ einfügen.
5. **„Stand heute“ und Schluss neu schreiben**, nicht anhängen: Datum, Anzahl Commits
   (`git rev-list --count HEAD`), Firmware-Patches (`firmware/patches/`), Statusliste je Modus (gefahren /
   eingebaut, nicht gefahren / nur Simulator), Absturzschutz. Der Schlussabsatz greift ein Leitmotiv auf
   (Maßband, Treppe, Python vs. C++) und endet mit einer trockenen Pointe. Die Fußnote (`<hr>` + Quellen)
   bleibt, Quellen ggf. ergänzen. `<title>`/`<h1>` („Zwei Wochen …“) bei längerem Zeitraum mit dem Nutzer
   abstimmen, nicht eigenmächtig ändern.
6. **Endkontrolle:** Rechtschreibung der neuen Abschnitte, jede Zahl gegen die Quelle, HTML gültig (nur
   `<h2>`, `<p>`, `<ul><li>`, `<strong>`, `<code>`, `<em>`; Zitate mit „…“ statt `<q>`). Kein neues CSS.
7. **Immer sofort auf den Rider einspielen** (Nutzerwunsch 2026-11-18, ohne Rückfrage): Rider-Adresse
   10.10.0.110 oder 192.168.53.110 probieren, alte Datei nach `/root/geschichte.html.bak-<Datum-Zeit>` sichern,
   per scp nach `/opt/xgonav/web/geschichte.html` (nginx-Alias `/xgonav/`), danach
   `chown aeinstein:aeinstein`, `file` (LF, UTF-8) und md5 gegen lokal prüfen. Kein Neustart nötig.
   Dem Nutzer die neuen Kapitel kurz nennen. **Commit nur auf Zuruf** (auf dem aktuellen Branch, nur die
   eigenen Dateien, Präfix „Geschichte: …“, keine Session-Zeile).

## Stil (aus dem bestehenden Text)

- **Perspektive:** „ich“ = Claude (alle Sitzungen zusammengefasst, auch der Firmware-Claude), „du“ = der Nutzer,
  „wir“ für Gemeinsames, „der Rider“/„er“ für den Roboter. Der Nutzer wird nie mit Namen genannt.
- **Ton:** trocken, selbstironisch, sarkastisch, aber nie gehässig. Der Spott trifft vor allem Claude (baut
  zuerst in Python, ist von C++ überrascht, glaubt eigenen Kennzahlen mehr als dem Bild), dann die Technik
  (Firmware, die selbst nicht weiß, was sie tut; Simulator ohne Treppen), den Nutzer höchstens liebevoll.
- **Pointen am Absatzende**, kurz und lakonisch, oft als eigener Satz:
  „Synthetische Bilder lügen nicht, sie sagen nur nichts.“ –
  „Ein Knopf, der den Flash beschreibt, ist kein Knopf, sondern eine Falle mit Beschriftung.“ –
  „Er hatte sich also vor allem umgesehen.“
- **Vergleiche** aus dem Alltag, sparsam (höchstens einer je Absatz): „wie ein Hund, der dem Parkett nicht
  traut“, „wie ein Auto mit Schluckauf“, „wie jemand, der sehr gern möchte“.
- **Fakten exakt:** Datum, Uhrzeit, Messwerte mit deutscher Schreibweise (0,5 ms; 21 866; 2,3 cm; −31°).
  „Übertrieben ist der Ton, nicht die Zahlen.“ Nichts beschönigen: Fehler von Claude offen benennen
  („Es war meine“), Ungefahrenes als ungefahren kennzeichnen.
- **Zitate:** wörtlich in „…“, Kleinschreibung und Tippfehler des Nutzers bleiben. Ein Zitat pro Gedanke,
  gern als Wendepunkt („Das Netz hatte recht“).
- **Sätze:** mittellang, einfache Hauptsätze, kein Fachjargon ohne Erklärung, Code-Namen in `<code>`. Keine
  Emojis, keine Ausrufezeichen, keine Superlative im Erzählton, keine Gedankenstrich-Ketten.
- **Leitmotive** wieder aufnehmen: Maßband gegen Vermutung, die Treppe und der Zaun, Python gegen C++,
  Rechenzeit am Rider, „nie gefahren“, der Nutzer als Bote zwischen zwei Claudes.
- **Deutsch mit korrekten Umlauten und ß**, auch im HTML (Datei ist UTF-8). Lagewinkel heißen Nick, Roll, Gier,
  nie „Neigung“.

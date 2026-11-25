//
// Fahrrahmen in mm/s und °/s (Firmware-Patches 0029–0032, ARBEITSPLAN.md Punkt 4, Stufe 7).
//
// Nur Text <-> Bytes, ohne Kernel-Aufrufe außer sscanf/strcmp: der Treiber (xgo-proc.c) und das Test-Harness
// (driver-test/km/test_main.c) nutzen dieselben Funktionen.
//
// /proc/XGORider/vel schreiben (ein Rahmen, Register 0x1a, 13 Byte little-endian):
//   "V A J W ALPHA JW"      Tempo-Sollwert: V mm/s, A mm/s² (≥ 0), J mm/s³ (≥ 0), W °/s (eine Nachkommastelle),
//                           ALPHA °/s², JW °/s³ (≥ 0); 0 bei A/J/ALPHA/JW = ohne diese Grenze
//   "acc A J W ALPHA JW"    Beschleunigungsbetrieb: A mm/s² mit Vorzeichen, ohne Tempo-Sollwert (bis ±v_max)
// /proc/XGORider/vel lesen (Register 0x1a, 12 Byte):
//   "aktiv beschl waechter grenzen v w v_max w_max timeout_ms", z. B. "1 0 0 1 250 -12.3 500 90.0 300"
// /proc/XGORider/vel_limits schreiben (Register 0x1b, 6 Byte): "V_MAX W_MAX TIMEOUT_MS" (W_MAX eine Nachkommastelle,
//   Wächter 0 = aus). Ohne diese Grenzen verwirft die Firmware jeden Fahrrahmen; xgonav schreibt sie beim Start.
//   Firmware 0035, 10 Byte: "V_MAX W_MAX TIMEOUT_MS EINHEIT_UM VORFAKTOR_PROMILLE" (µm/s je Firmware-Einheit,
//   z. B. 3900; Vorfaktor des Bestandswegs, z. B. 550; je 0 = Standard der Firmware). Ältere Firmware verwirft 10 Byte.
//   Firmware 0037, 14 Byte: dazu "KI_E4 IMAX_CENTI" (ki × 10000 in ° je Einheit·s, i_max in 0,01°; 0 = Konsolenwert).
//   Firmware 0038, 16 Byte: dazu "KV_E4" (Tempo-Vorsteuerung des Soll-Nicks × 10000 in ° je Einheit; 0 = aus).
//
#ifndef XGO_VEL_H
#define XGO_VEL_H

#define XGO_VEL_FRAME_LEN 13
#define XGO_VEL_LIMITS_LEN 16      /* Puffergröße; gesendet werden 6, 10, 14 oder 16 Byte (vel_limits_encode) */
#define XGO_VEL_LIMITS_LEN_CTL 14
#define XGO_VEL_LIMITS_LEN_EXT 10
#define XGO_VEL_LIMITS_LEN_OLD 6
#define XGO_VEL_DIAG_LEN 12
#define XGO_VEL_FLAG_NO_SPEED 1

static void vel_put16(uint8_t *d, int v) { d[0] = (uint8_t)(v & 0xff); d[1] = (uint8_t)((v >> 8) & 0xff); }
static int vel_s16(const uint8_t *d) { return (int16_t)(uint16_t)(d[0] | (d[1] << 8)); }
static int vel_u16(const uint8_t *d) { return (uint16_t)(d[0] | (d[1] << 8)); }

// "-12.3" -> -123, "5" -> 50, "0.5" -> 5; höchstens eine Nachkommastelle. 0 = ok.
static int vel_parse_tenths(const char *s, int *out) {
	int neg = 0, whole = 0, frac = 0, digits = 0;
	if (*s == '-' || *s == '+') neg = *s++ == '-';
	if (*s < '0' || *s > '9') return -1;
	while (*s >= '0' && *s <= '9') {
		whole = whole * 10 + (*s++ - '0');
		if (whole > 100000) return -1;
	}
	if (*s == '.') {
		s++;
		while (*s >= '0' && *s <= '9') {
			if (digits++ == 0) frac = *s - '0';
			else if (*s != '0') return -1;   // mehr als eine Nachkommastelle
			s++;
		}
	}
	if (*s != '\0') return -1;
	*out = (whole * 10 + frac) * (neg ? -1 : 1);
	return 0;
}

static int vel_in(int v, int lo, int hi) { return v >= lo && v <= hi; }

// Text -> Rahmen 0x1a. 0 = ok, -1 = ungültig.
static int vel_encode(const char *text, uint8_t out[XGO_VEL_FRAME_LEN]) {
	char wtok[16];
	int v = 0, a, j, w10, alpha, jw, n, accel = 0;
	if (sscanf(text, "acc %d %d %15s %d %d%n", &a, &j, wtok, &alpha, &jw, &n) == 5) accel = 1;
	else if (sscanf(text, "%d %d %d %15s %d %d%n", &v, &a, &j, wtok, &alpha, &jw, &n) != 6) return -1;
	while (text[n] == ' ' || text[n] == '\n' || text[n] == '\t') n++;
	if (text[n] != '\0') return -1;
	if (vel_parse_tenths(wtok, &w10)) return -1;
	if (!vel_in(v, -32768, 32767) || !vel_in(w10, -32768, 32767)) return -1;
	if (!vel_in(a, accel ? -32768 : 0, 32767)) return -1;
	if (!vel_in(j, 0, 65535) || !vel_in(alpha, 0, 65535) || !vel_in(jw, 0, 65535)) return -1;
	out[0] = accel ? XGO_VEL_FLAG_NO_SPEED : 0;
	vel_put16(out + 1, v); vel_put16(out + 3, a); vel_put16(out + 5, j);
	vel_put16(out + 7, w10); vel_put16(out + 9, alpha); vel_put16(out + 11, jw);
	return 0;
}

// Text -> Grenzen 0x1b. Länge (6, 10, 14 oder 16 Byte) = ok, -1 = ungültig.
static int vel_limits_encode(const char *text, uint8_t out[XGO_VEL_LIMITS_LEN]) {
	char wtok[16];
	int vmax, wmax10, ms, n, unit = 0, scale = 0, ki = 0, imax = 0, kv = 0, len = XGO_VEL_LIMITS_LEN_OLD;
	if (sscanf(text, "%d %15s %d %d %d %d %d %d%n", &vmax, wtok, &ms, &unit, &scale, &ki, &imax, &kv, &n) == 8) len = XGO_VEL_LIMITS_LEN;
	else if (sscanf(text, "%d %15s %d %d %d %d %d%n", &vmax, wtok, &ms, &unit, &scale, &ki, &imax, &n) == 7) len = XGO_VEL_LIMITS_LEN_CTL;
	else if (sscanf(text, "%d %15s %d %d %d%n", &vmax, wtok, &ms, &unit, &scale, &n) == 5) len = XGO_VEL_LIMITS_LEN_EXT;
	else if (sscanf(text, "%d %15s %d%n", &vmax, wtok, &ms, &n) != 3) return -1;
	while (text[n] == ' ' || text[n] == '\n' || text[n] == '\t') n++;
	if (text[n] != '\0') return -1;
	if (vel_parse_tenths(wtok, &wmax10)) return -1;
	if (!vel_in(vmax, 0, 65535) || !vel_in(wmax10, 0, 65535) || !vel_in(ms, 0, 65535)) return -1;
	if (!vel_in(unit, 0, 65535) || !vel_in(scale, 0, 2000)) return -1;
	if (!vel_in(ki, 0, 10000) || !vel_in(imax, 0, 1000) || !vel_in(kv, 0, 2000)) return -1;
	vel_put16(out, vmax); vel_put16(out + 2, wmax10); vel_put16(out + 4, ms);
	vel_put16(out + 6, unit); vel_put16(out + 8, scale); vel_put16(out + 10, ki); vel_put16(out + 12, imax);
	vel_put16(out + 14, kv);
	return len;
}

// Zustand 0x1a (12 Byte) -> Text. Zehntel ohne Fließkomma.
static int vel_format_diag(const uint8_t d[XGO_VEL_DIAG_LEN], char *buf, size_t len) {
	int w10 = vel_s16(d + 4), wm10 = vel_u16(d + 8);
	return snprintf(buf, len, "%d %d %d %d %d %s%d.%d %d %d.%d %d\n",
	                d[0] & 1, (d[0] >> 1) & 1, (d[0] >> 2) & 1, (d[0] >> 3) & 1, vel_s16(d + 2),
	                w10 < 0 ? "-" : "", (w10 < 0 ? -w10 : w10) / 10, (w10 < 0 ? -w10 : w10) % 10,
	                vel_u16(d + 6), wm10 / 10, wm10 % 10, vel_u16(d + 10));
}

#endif // XGO_VEL_H

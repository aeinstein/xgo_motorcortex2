/* Harness: echter Treibercode (xgo-serial.c + IMU-Teil aus xgo-drv.c) gegen ESP32-Emulator. */
#include "../kshim.h"
const char *g_serial_path;
#include "xgo-serial.c"
#include "imu_extract.c"
#include "xgo-vel.h"   /* Fahrrahmen 0x1a/0x1b (Firmware-Patch 08-velocity), dieselben Funktionen wie /proc/XGORider/vel */
#include <math.h>

/* Text -> Bytes der Fahrrahmen prüfen, ohne Emulator */
static int vel_case(const char *text, int want_ok, const uint8_t *want, int lim) {
	uint8_t out[16] = {0};   /* Rahmen 13, Grenzen 14 Byte */
	int r = lim ? vel_limits_encode(text, out) : vel_encode(text, out);
	int n = lim ? lim : XGO_VEL_FRAME_LEN;   /* lim = erwartete Länge der Grenzen */
	int ok = want_ok ? (r == (lim ? n : 0) && memcmp(out, want, n) == 0) : r < 0;
	if (!ok) printf("VEL FEHLER '%s' (r %d)\n", text, r);
	return !ok;
}
static int test_vel(void) {
	int bad = 0;
	const uint8_t f1[] = {0, 0xfa, 0x00, 0xf4, 0x01, 0xc4, 0x09, 0x7b, 0x00, 0x78, 0x00, 0x58, 0x02};   /* 250 500 2500 12.3 120 600 */
	const uint8_t f2[] = {1, 0, 0, 0xd4, 0xfe, 0xd0, 0x07, 0xce, 0xff, 0, 0, 0, 0};                     /* acc -300 2000 -5 0 0 */
	const uint8_t f3[] = {0, 0x9c, 0xff, 0, 0, 0, 0, 0x05, 0x00, 0, 0, 0, 0};                             /* -100 0 0 0.50 0 0 */
	bad += vel_case("250 500 2500 12.3 120 600", 1, f1, 0);
	bad += vel_case("250 500 2500 12.3 120 600\n", 1, f1, 0);
	bad += vel_case("acc -300 2000 -5 0 0", 1, f2, 0);
	bad += vel_case("-100 0 0 0.50 0 0", 1, f3, 0);
	bad += vel_case("250 -500 0 0 0 0", 0, NULL, 0);        /* negatives a nur im Beschleunigungsbetrieb */
	bad += vel_case("1 2 3 4.25 5 6", 0, NULL, 0);          /* zwei Nachkommastellen */
	bad += vel_case("1 2 3 4 5", 0, NULL, 0);
	bad += vel_case("1 2 3 4 5 6 7", 0, NULL, 0);
	bad += vel_case("40000 0 0 0 0 0", 0, NULL, 0);
	bad += vel_case("acc 1 2 x 4 5", 0, NULL, 0);
	bad += vel_case("1 2 3 4 -5 6", 0, NULL, 0);
	{
		const uint8_t k1[] = {2, 0x2e, 0x00, 0x80, 0x00, 0x81, 0x02, 0xd0, 0x07, 0x68, 0x01, 0x08, 0x07};   /* kap 46 128 641 2000 360 1800 */
		const uint8_t k2[] = {2, 0xd2, 0xff, 0, 0, 0, 0, 0x30, 0xf8, 0, 0, 0, 0};                         /* kap -46 0 0 -2000 0 0 */
		bad += vel_case("kap 46 128 641 2000 360 1800", 1, k1, 0);
		bad += vel_case("kap -46 0 0 -2000 0 0\n", 1, k2, 0);
		bad += vel_case("kap 46 128 641 2.5 360 1800", 0, NULL, 0);   /* κ ganzzahlig in 0,001 1/m */
		bad += vel_case("kap 46 128 641 40000 360 1800", 0, NULL, 0);
		bad += vel_case("kap 46 -1 641 2000 360 1800", 0, NULL, 0);
		const uint8_t c1[] = {0x14, 0, 0, 0x80, 0x00, 0x81, 0x02, 0x84, 0x03, 0x68, 0x01, 0x08, 0x07};   /* kurs 1 0 128 641 90.0 360 1800 */
		const uint8_t c2[] = {0x04, 0x2e, 0x00, 0, 0, 0, 0, 0x83, 0xff, 0, 0, 0, 0};                     /* kurs 0 46 0 0 -12.5 0 0 */
		bad += vel_case("kurs 1 0 128 641 90.0 360 1800", 1, c1, 0);
		bad += vel_case("kurs 0 46 0 0 -12.5 0 0\n", 1, c2, 0);
		bad += vel_case("kurs 2 0 128 641 90.0 360 1800", 0, NULL, 0);   /* Wechselbit nur 0/1 */
		bad += vel_case("kurs 1 0 128 641 90.05 360 1800", 0, NULL, 0);
		bad += vel_case("kurs 1 0 128 641 90.0 360", 0, NULL, 0);
	}
	{   /* Grenzen 0x1b, 14 Byte */
		const uint8_t l1[] = {0xf4, 0x01, 0x89, 0x03, 0x2c, 0x01, 0x3c, 0x0f, 0x2c, 0x01, 0x58, 0x02, 0xfa, 0x00};   /* 500 90.5 300 3900 300 600 250 */
		bad += vel_case("500 90.5 300 3900 300 600 250", 1, l1, 14);
		bad += vel_case("500 90.5 300 3900 300 600 250\n", 1, l1, 14);
		bad += vel_case("500 90.5 300", 0, NULL, 14);                   /* altes Format */
		bad += vel_case("500 -1 300 3900 300 600 250", 0, NULL, 14);
		bad += vel_case("500 90 300 -1 300 600 250", 0, NULL, 14);
		bad += vel_case("500 90 300 3900 10001 600 250", 0, NULL, 14);
		bad += vel_case("500 90 300 3900 300 1001 250", 0, NULL, 14);
		bad += vel_case("500 90 300 3900 300 600 2001", 0, NULL, 14);
		bad += vel_case("500 90 300 3900 300 600 250 1", 0, NULL, 14);
	}
	{
		const uint8_t d[] = {9, 0, 0xfa, 0x00, 0x85, 0xff, 0xf4, 0x01, 0x84, 0x03, 0x2c, 0x01};
		const uint8_t e[] = {1, 0, 0, 0, 0xfb, 0xff, 0, 0, 0, 0, 0, 0};
		char t[80];
		vel_format_diag(d, t, sizeof(t));
		if (strcmp(t, "1 0 0 1 250 -12.3 500 90.0 300 0 0 0 0\n")) { printf("VEL FEHLER Zustand '%s'\n", t); bad++; }
		vel_format_diag(e, t, sizeof(t));
		if (strcmp(t, "1 0 0 0 0 -0.5 0 0.0 0 0 0 0 0\n")) { printf("VEL FEHLER Zustand '%s'\n", t); bad++; }
		const uint8_t c[] = {0x69, 3, 0, 0, 0, 0, 0x58, 0x02, 0xe8, 0x03, 0x2c, 0x01};
		vel_format_diag(c, t, sizeof(t));
		if (strcmp(t, "1 0 0 1 0 0.0 600 100.0 300 0 3 1 1\n")) { printf("VEL FEHLER Zustand '%s'\n", t); bad++; }
		const uint8_t k[] = {0x19, 1, 0x2e, 0x00, 0x2e, 0x00, 0x58, 0x02, 0xb8, 0x0b, 0x2c, 0x01};
		vel_format_diag(k, t, sizeof(t));
		if (strcmp(t, "1 0 0 1 46 4.6 600 300.0 300 1 1 0 0\n")) { printf("VEL FEHLER Zustand '%s'\n", t); bad++; }
	}
	printf("VELPARSE %s\n", bad ? "FEHLER" : "ok");
	return bad;
}

/* mit --vel im Emulator: Grenzen + Rahmen wie /proc schicken, Zustand zurücklesen */
static void vel_roundtrip(void) {
	static const char *lim = "450 72.5 300 3900 300 600 250";   /* 14 Byte */
	static const char *frames[] = {"250 500 2500 12.3 120 600", "acc -300 2000 -5 0 0", "kap 46 128 641 2000 360 1800",
	                               "kurs 1 0 128 641 90.0 360 1800", "-120 800 4000 -30.0 90 400"};
	uint8_t b[16];   /* Rahmen 13, Grenzen bis 14 Byte */
	char hex[64], t[80];
	int ln = vel_limits_encode(lim, b);
	write_serial_data(XGO_VEL_LIMITS, (char *)b, ln);
	for (int i = 0; i < ln; i++) sprintf(hex + 2 * i, "%02x", b[i]);
	printf("VELLIM %s\n", hex);
	for (size_t k = 0; k < sizeof(frames) / sizeof(frames[0]); k++) {
		vel_encode(frames[k], b);
		write_serial_data(XGO_VEL, (char *)b, XGO_VEL_FRAME_LEN);
		for (int i = 0; i < XGO_VEL_FRAME_LEN; i++) sprintf(hex + 2 * i, "%02x", b[i]);
		printf("VELSENT %s\n", hex);
		msleep(20);
	}
	printf("VELWANT 1 0 0 1 -120 -30.0 450 72.5 300 0 3 0 0\n");
	for (int tries = 0; tries < 5; tries++) {
		uint8_t d[XGO_VEL_DIAG_LEN];
		if (read_addr_sync(XGO_VEL, sizeof(d), d, 100) >= (int)sizeof(d)) {
			vel_format_diag(d, t, sizeof(t));
			t[strcspn(t, "\n")] = 0;
			printf("VELDIAG %s\n", t);
			return;
		}
	}
	printf("VELDIAG Fehler\n");
}

static atomic_int g_stop_aux;
static int n_main_ok, n_main_fail, n_writes;

static void *main_like(void *a) {   /* wie loop(): Batterie/State/Yaw alle 200 ms */
	(void)a;
	while (!atomic_load(&g_stop_aux)) {
		uint8_t b[2];
		if (read_addr_sync(XGO_BATTERY, 1, b, 50) >= 1) n_main_ok++; else n_main_fail++;
		if (read_addr_sync(XGO_STATE, 1, b, 50) >= 1) n_main_ok++; else n_main_fail++;
		if (read_addr_sync(XGO_YAW_INT, 2, b, 50) >= 2) n_main_ok++; else n_main_fail++;
		{ char ip[4] = {192, 168, 53, 110}; write_serial_data(XGO_PI_IP, ip, 4); }   /* wie pi_ip_tick() */
		msleep(200);
	}
	return NULL;
}

static void *writer(void *a) {      /* xgonav schreibt speed_x/speed_z mit 20 Hz */
	(void)a;
	uint8_t v = 140;
	while (!atomic_load(&g_stop_aux)) {
		write_serial_data(XGO_VX, (char *)&v, 1);
		write_serial_data(XGO_VYAW, (char *)&v, 1);
		n_writes += 2;
		msleep(50);
	}
	return NULL;
}

/* BLE-Tunnel (Firmware-Patch 01-ble): wie xgo-ble jeden Datensatz holen und die Daten zurückschicken */
static int n_tun_data, n_tun_conn, n_tun_bytes;
static void *tunnel_echo(void *a) {
	(void)a;
	while (!atomic_load(&g_stop_aux)) {
		uint8_t rec[XGO_TUNNEL_MAX];
		int n = ble_rx_pop(rec);
		if (!n) { msleep(2); continue; }
		if (rec[0] == 0) { n_tun_data++; n_tun_bytes += n - 1; ble_tx(rec + 1, n - 1); }
		else n_tun_conn++;
	}
	return NULL;
}

static int test_f32(void) {
	int bad = 0;
	srand(1);
	float specials[] = {0.0f, -0.0f, 1.0f, -1.0f, 0.0005f, -0.0005f, 0.0015f, 359.999f, -180.0f, 1e-30f, 123.4567f};
	for (int i = 0; i < 200000; i++) {
		float f = i < 11 ? specials[i] : ((float)rand() / RAND_MAX - 0.5f) * 800.0f;
		uint8_t b[4]; memcpy(b, &f, 4);   /* x86/ARM: little endian wie vom ESP32 */
		s32 got = f32le_to_milli(b);
		long want = lround((double)f * 1000.0);
		if (labs(got - want) > 1) { if (bad++ < 5) printf("f32 FEHLER %g -> %d (soll %ld)\n", f, got, want); }
	}
	printf("F32 %s (200000 Werte)\n", bad ? "FEHLER" : "ok");
	return bad;
}

int main(int argc, char **argv) {
	g_serial_path = argv[1];
	double secs = atof(argv[2]);
	XGO_IMU_HZ = atoi(argv[3]);
	if (argc > 4) XGO_IMU_STREAM_HZ = atoi(argv[4]);   /* Strom (Firmware-Patch 02-pi-interface) einschalten */
	if (test_f32()) return 2;
	if (test_vel()) return 2;

	if (open_serial_port()) { printf("open failed\n"); return 1; }
	char fw[11] = {0};
	int r = read_addr_sync(XGO_FIRMWARE_VERSION, 10, (uint8_t *)fw, 500);
	printf("FW %d '%.10s'\n", r, fw);
	imu_detect_combo();
	printf("BLOCK %d\n", combo_supported ? 3 : 0);
	printf("WHEELS %d\n", combo_supported);
	if (getenv("XGO_TEST_VEL")) vel_roundtrip();

	imu_thread = kthread_run(imu_loop, NULL, "imu");
	pthread_t tm, tw, tt;
	pthread_create(&tm, NULL, main_like, NULL);
	pthread_create(&tw, NULL, writer, NULL);
	pthread_create(&tt, NULL, tunnel_echo, NULL);

	/* wie Userspace: imu_log periodisch lesen, neue Proben ausgeben */
	u32 last = 0, wlast = 0;
	u64 t_end = ktime_get_ns() + (u64)(secs * 1e9);
	while (ktime_get_ns() < t_end) {
		msleep(100);
		struct imu_sample copy[IMU_LOG_LEN]; u32 seq;
		spin_lock(&imu_lock); memcpy(copy, imu_log, sizeof(copy)); seq = imu_seq; spin_unlock(&imu_lock);
		u32 first = seq > IMU_LOG_LEN - 1 ? seq - (IMU_LOG_LEN - 1) : 1;
		if (last + 1 > first) first = last + 1;
		for (u32 i = first; seq && i <= seq; i++) {
			struct imu_sample *p = &copy[i % IMU_LOG_LEN];
			if (p->seq == i) printf("S %u %llu %d %d %d\n", p->seq, (unsigned long long)p->t_ns, p->roll, p->pitch, p->yaw);
		}
		/* Räder genauso (wheel_log) */
		struct wheel_sample wc[WHEEL_LOG_LEN]; u32 ws;
		spin_lock(&imu_lock); memcpy(wc, wheel_log, sizeof(wc)); ws = wheel_seq; spin_unlock(&imu_lock);
		u32 wf = ws > WHEEL_LOG_LEN - 1 ? ws - (WHEEL_LOG_LEN - 1) : 1;
		if (wlast + 1 > wf) wf = wlast + 1;
		for (u32 i = wf; ws && i <= ws; i++) {
			struct wheel_sample *p = &wc[i % WHEEL_LOG_LEN];
			if (p->seq == i) printf("W %u %llu %d %d %u\n", p->seq, (unsigned long long)p->t_ns, p->t11, p->t21, p->fw_ms);
		}
		wlast = ws;
		last = seq;
	}

	/* Exit-Reihenfolge wie imu_proc_exit: Reader zuerst, solange abgefragt wird */
	u64 t0 = ktime_get_ns();
	stop_read_loop();
	u64 t1 = ktime_get_ns();
	kthread_stop(imu_thread);
	atomic_store(&g_stop_aux, 1);
	pthread_join(tm, NULL); pthread_join(tw, NULL); pthread_join(tt, NULL);
	u64 t2 = ktime_get_ns();
	printf("EXIT reader %.1f ms, rest %.1f ms\n", (t1 - t0) / 1e6, (t2 - t1) / 1e6);
	printf("STATS frames %d bad %d timeouts %d unsolicited %d main_ok %d main_fail %d writes %d\n",
	       atomic_read(&stat_frames), atomic_read(&stat_bad), atomic_read(&stat_timeouts),
	       atomic_read(&stat_unsolicited), n_main_ok, n_main_fail, n_writes);
	printf("STREAM pakete %d luecken %d versatz_us %lld\n", atomic_read(&stat_stream),
	       atomic_read(&stat_stream_gaps), stream_off_ok ? (long long)(stream_off_ns / 1000) : 0LL);
	printf("TUNNEL saetze %d bytes %d meldungen %d verworfen %d gesendet %d\n", n_tun_data, n_tun_bytes, n_tun_conn,
	       atomic_read(&stat_tunnel_drop), atomic_read(&stat_tunnel_tx));
	filp_close(serial_file, NULL);
	return 0;
}

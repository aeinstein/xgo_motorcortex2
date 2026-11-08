/* Harness: echter Treibercode (xgo-serial.c + IMU-Teil aus xgo-drv.c) gegen ESP32-Emulator. */
#include "../kshim.h"
const char *g_serial_path;
#include "xgo-serial.c"
#include "imu_extract.c"
#include <math.h>

static atomic_int g_stop_aux;
static int n_main_ok, n_main_fail, n_writes;

static void *main_like(void *a) {   /* wie loop(): Batterie/State/Yaw alle 200 ms */
	(void)a;
	while (!atomic_load(&g_stop_aux)) {
		uint8_t b[2];
		if (read_addr_sync(XGO_BATTERY, 1, b, 50) >= 1) n_main_ok++; else n_main_fail++;
		if (read_addr_sync(XGO_STATE, 1, b, 50) >= 1) n_main_ok++; else n_main_fail++;
		if (read_addr_sync(XGO_YAW_INT, 2, b, 50) >= 2) n_main_ok++; else n_main_fail++;
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
	if (argc > 4) XGO_IMU_STREAM_HZ = atoi(argv[4]);   /* Strom (Firmware 0014) einschalten */
	if (test_f32()) return 2;

	if (open_serial_port()) { printf("open failed\n"); return 1; }
	char fw[11] = {0};
	int r = read_addr_sync(XGO_FIRMWARE_VERSION, 10, (uint8_t *)fw, 500);
	printf("FW %d '%.10s'\n", r, fw);
	imu_detect_block_mode();
	printf("BLOCK %d\n", imu_block_mode);
	printf("WHEELS %d\n", wheels_supported);

	imu_thread = kthread_run(imu_loop, NULL, "imu");
	pthread_t tm, tw;
	pthread_create(&tm, NULL, main_like, NULL);
	pthread_create(&tw, NULL, writer, NULL);

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
	pthread_join(tm, NULL); pthread_join(tw, NULL);
	u64 t2 = ktime_get_ns();
	printf("EXIT reader %.1f ms, rest %.1f ms\n", (t1 - t0) / 1e6, (t2 - t1) / 1e6);
	printf("STATS frames %d bad %d timeouts %d unsolicited %d main_ok %d main_fail %d writes %d\n",
	       atomic_read(&stat_frames), atomic_read(&stat_bad), atomic_read(&stat_timeouts),
	       atomic_read(&stat_unsolicited), n_main_ok, n_main_fail, n_writes);
	printf("STREAM pakete %d luecken %d versatz_us %lld\n", atomic_read(&stat_stream),
	       atomic_read(&stat_stream_gaps), stream_off_ok ? (long long)(stream_off_ns / 1000) : 0LL);
	filp_close(serial_file, NULL);
	return 0;
}

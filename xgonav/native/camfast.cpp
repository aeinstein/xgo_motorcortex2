// Kamera direkt über libcamera (statt picamera2): das fertige ISP-Bild wird in C++ aus dem DMA-Puffer am Stück in
// einen gecachten Puffer kopiert und von dort entzerrt (lens_remap aus linsefast.cpp); es landet so im Ausgabebild
// von Python, ohne dass die Pixel vorher durch Python laufen. Eigene Bibliothek libxgocam.so (make -C xgonav/native cam), damit libxgofast.so kein libcamera braucht.
//
// Einstellungen wie picamera2.create_video_configuration (main RGB888 = BGR im Speicher, Farbraum SMPTE170M,
// NoiseReductionMode Fast, Raw-Strom für den Sensormodus, FrameDurationLimits, ScalerCrop), damit die Bilder
// dieselben bleiben, auf denen FloorNet trainiert ist.
//
// Fäden: libcamera meldet fertige Requests in seinem eigenen Faden (done); dort wird nur das neueste gemerkt, ein
// älteres sofort wieder eingestellt (nur der neueste Frame zählt). cam_get läuft im Kamera-Faden von Python (ctypes
// gibt das GIL frei), entzerrt, liest die Metadaten und stellt den Request wieder ein. cam_get, cam_start und
// cam_stop kommen alle aus diesem einen Faden (camera.FrameSource._run), nie gleichzeitig.

#include <libcamera/libcamera.h>
#include <libcamera/version.h>

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

extern "C" int lens_remap(const uint8_t* src, int w, int h, int stride, int cn, const int16_t* map1,
                          const uint16_t* map2, uint8_t* dst, int dw, int dh, int dstride);

#define XGO_LC_AB(maj, min) (LIBCAMERA_VERSION_MAJOR > (maj) || \
                             (LIBCAMERA_VERSION_MAJOR == (maj) && LIBCAMERA_VERSION_MINOR >= (min)))

using namespace libcamera;

namespace {
constexpr int kCamAbi = 2;   // bei Änderung von Funktionen oder CamMeta erhöhen (camera.py prüft das)

struct CamMeta {             // gleiche Reihenfolge wie camera._CamMeta
    int64_t ts_ns;           // SensorTimestamp (CLOCK_MONOTONIC wie bisher), 0 = fehlt
    int64_t frame_us;        // FrameDuration, -1 = fehlt
    int32_t exp_us;          // ExposureTime, -1 = fehlt
    float gain;              // AnalogueGain, 0 = fehlt
    float awb[2];            // ColourGains
    int32_t has_awb;
    int32_t crop[4];         // ScalerCrop x, y, w, h
    int32_t has_crop;
    int64_t dropped;         // seit dem Start verworfene fertige Bilder (Python war noch beschäftigt)
};
static_assert(sizeof(CamMeta) == 64 && offsetof(CamMeta, crop) == 36 && offsetof(CamMeta, dropped) == 56,
              "CamMeta passt nicht zu camera._CamMeta");

struct Mapped {
    const uint8_t* p = nullptr;   // Anfang der Ebene 0
    void* base = nullptr;
    size_t len = 0;
    int fd = -1;
};

void dma_sync(int fd, uint64_t flags) {
    dma_buf_sync s{};
    s.flags = flags;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);   // Fehler egal: ohne Cache-Pflege liest man nur evtl. veraltete Zeilen
}

struct Cam {
    std::unique_ptr<CameraManager> cm;
    std::shared_ptr<Camera> cam;
    std::unique_ptr<CameraConfiguration> conf;
    std::unique_ptr<FrameBufferAllocator> alloc;
    Stream* main = nullptr;
    Stream* raw = nullptr;
    std::vector<std::unique_ptr<Request>> reqs;
    std::map<const FrameBuffer*, Mapped> maps;
    int w = 0, h = 0, stride = 0, raw_w = 0, raw_h = 0;
    std::vector<int16_t> m1;     // Entzerrtabellen (leer = nur kopieren)
    std::vector<uint16_t> m2;
    std::vector<uint8_t> roh;    // gecachte Kopie des Kamerabilds (nur cam_get, ein Faden)

    std::mutex mu;
    std::condition_variable cv;
    Request* ready = nullptr;    // neuestes fertiges Bild, noch nicht abgeholt
    bool running = false;
    bool acquired = false;
    ControlList cur{controls::controls};       // alle gesetzten Werte, gehen bei jedem Start mit
    ControlList pending{controls::controls};   // Änderungen für den nächsten eingestellten Request
    int64_t dropped = 0;
    // Zeitmessung je Abschnitt in cam_get (ns, aufsummiert; cam_zeiten liest und setzt zurück), nur der Kamera-Faden
    int64_t t_sync = 0, t_kopie = 0, t_remap = 0, t_meta = 0, n_bilder = 0;

    void requeue(Request* r) {   // unter mu
        r->reuse(Request::ReuseBuffers);
        if (!pending.empty()) {
            r->controls().merge(pending);
            pending.clear();
        }
        cam->queueRequest(r);
    }

    void done(Request* r) {
        if (r->status() == Request::RequestCancelled) return;
        std::lock_guard<std::mutex> lk(mu);
        if (!running) return;
        if (ready) {
            requeue(ready);
            ++dropped;
        }
        ready = r;
        cv.notify_all();
    }

    template <typename T, typename V>
    void set(const Control<T>& id, const V& v) {   // unter mu
        cur.set(id, v);
        pending.set(id, v);
    }

    ~Cam() {
        if (cam) {
            if (running) cam->stop();
            cam->requestCompleted.disconnect(this);
        }
        for (auto& kv : maps)
            if (kv.second.base) munmap(kv.second.base, kv.second.len);
        reqs.clear();
        alloc.reset();
        if (cam && acquired) cam->release();
        cam.reset();
        conf.reset();
        if (cm) cm->stop();
    }
};

int64_t jetzt_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

void fehler(char* err, int n, const char* s) {
    if (err && n > 0) std::snprintf(err, static_cast<size_t>(n), "%s", s);
}
}  // namespace

extern "C" {

int cam_abi(void) { return kCamAbi; }

// Kamera öffnen und einrichten (noch nicht gestartet). raw_w/raw_h 0 = Sensormodus wählt libcamera, crop NULL =
// ganzer Modus. Rückgabe Handle oder NULL (Grund in err).
void* cam_open(int w, int h, int raw_w, int raw_h, int64_t frame_us, const int32_t* crop, int nbuf, char* err,
               int errlen) {
    std::unique_ptr<Cam> c(new Cam);
    c->cm.reset(new CameraManager);
    if (c->cm->start()) { fehler(err, errlen, "CameraManager startet nicht"); return nullptr; }
    if (c->cm->cameras().empty()) { fehler(err, errlen, "keine Kamera gefunden"); return nullptr; }
    c->cam = c->cm->cameras()[0];
    if (c->cam->acquire()) { fehler(err, errlen, "Kamera belegt (anderer Prozess?)"); return nullptr; }
    c->acquired = true;

    const bool mit_raw = raw_w > 0 && raw_h > 0;
    c->conf = mit_raw ? c->cam->generateConfiguration({StreamRole::VideoRecording, StreamRole::Raw})
                      : c->cam->generateConfiguration({StreamRole::VideoRecording});
    if (!c->conf) { fehler(err, errlen, "generateConfiguration scheitert"); return nullptr; }
    StreamConfiguration& sm = c->conf->at(0);
    sm.pixelFormat = formats::RGB888;
    sm.size = Size(static_cast<unsigned>(w), static_cast<unsigned>(h));
    sm.bufferCount = static_cast<unsigned>(nbuf);
    // wie picamera2 unter 1280x720: SMPTE170M (libcamera macht daraus für RGB SMPTE170M/Rec709/None/Full; am Rider
    // 2026-11-19 im Log beider Wege verglichen). sYCC ergäbe sRGB, andere Bilder als im FloorNet-Training.
    sm.colorSpace = ColorSpace::Smpte170m;
    if (mit_raw) {
        StreamConfiguration& sr = c->conf->at(1);
        sr.size = Size(static_cast<unsigned>(raw_w), static_cast<unsigned>(raw_h));
        sr.bufferCount = static_cast<unsigned>(nbuf);
#if XGO_LC_AB(0, 2)
        SensorConfiguration sc;          // wie picamera2: Sensormodus aus der Raw-Größe, 10 Bit (OV5647)
        sc.bitDepth = 10;
        sc.outputSize = sr.size;
        c->conf->sensorConfig = sc;
#endif
    }
    if (c->conf->validate() == CameraConfiguration::Invalid) { fehler(err, errlen, "Konfiguration ungültig"); return nullptr; }
    if (sm.pixelFormat != formats::RGB888 || sm.size != Size(static_cast<unsigned>(w), static_cast<unsigned>(h))) {
        fehler(err, errlen, "main-Strom wurde angepasst (Format oder Größe)");
        return nullptr;
    }
    if (c->cam->configure(c->conf.get())) { fehler(err, errlen, "configure scheitert"); return nullptr; }
    c->main = sm.stream();
    c->w = w;
    c->h = h;
    c->stride = static_cast<int>(sm.stride);
    if (mit_raw) {
        c->raw = c->conf->at(1).stream();
        c->raw_w = static_cast<int>(c->conf->at(1).size.width);
        c->raw_h = static_cast<int>(c->conf->at(1).size.height);
    }

    c->alloc.reset(new FrameBufferAllocator(c->cam));
    for (const StreamConfiguration& s : *c->conf)
        if (c->alloc->allocate(s.stream()) < 0) { fehler(err, errlen, "Puffer nicht anlegbar"); return nullptr; }
    const auto& bm = c->alloc->buffers(c->main);
    size_t n = bm.size();
    if (c->raw) n = std::min(n, c->alloc->buffers(c->raw).size());
    for (size_t i = 0; i < n; ++i) {
        std::unique_ptr<Request> r = c->cam->createRequest(i);
        if (!r || r->addBuffer(c->main, bm[i].get())) { fehler(err, errlen, "Request nicht anlegbar"); return nullptr; }
        if (c->raw && r->addBuffer(c->raw, c->alloc->buffers(c->raw)[i].get())) {
            fehler(err, errlen, "Raw-Puffer nicht anhängbar");
            return nullptr;
        }
        // main einmal abbilden: ganzer dmabuf, Ebene 0 ab ihrem Versatz
        const FrameBuffer::Plane& pl = bm[i]->planes()[0];
        Mapped m;
        m.fd = pl.fd.get();
        m.len = static_cast<size_t>(lseek(m.fd, 0, SEEK_END));
        if (m.len < pl.offset + pl.length) m.len = pl.offset + pl.length;
        m.base = mmap(nullptr, m.len, PROT_READ, MAP_SHARED, m.fd, 0);
        if (m.base == MAP_FAILED) { fehler(err, errlen, "mmap scheitert"); return nullptr; }
        m.p = static_cast<const uint8_t*>(m.base) + pl.offset;
        c->maps[bm[i].get()] = m;
        c->reqs.push_back(std::move(r));
    }

    // Span-Typ aus der Control selbst: libcamera::Span (bis 0.7) oder std::span (ab 0.8)
    using FdT = typename std::remove_reference_t<decltype(controls::FrameDurationLimits)>::type;
    const int64_t fd[2] = {frame_us, frame_us};
    c->cur.set(controls::FrameDurationLimits, FdT(fd));
    c->cur.set(controls::draft::NoiseReductionMode, controls::draft::NoiseReductionModeFast);
    if (crop) c->cur.set(controls::ScalerCrop, Rectangle(crop[0], crop[1], static_cast<unsigned>(crop[2]),
                                                         static_cast<unsigned>(crop[3])));
    c->cam->requestCompleted.connect(c.get(), &Cam::done);
    return c.release();
}

// Entzerrtabellen (Festkomma wie cv2.convertMaps CV_16SC2) für das Zielbild dw x dh = main-Größe; map1 NULL = aus.
int cam_set_lens(void* hdl, const int16_t* map1, const uint16_t* map2, int dw, int dh) {
    Cam* c = static_cast<Cam*>(hdl);
    if (!map1) { c->m1.clear(); c->m2.clear(); return 0; }
    if (dw != c->w || dh != c->h || !map2) return -1;
    const size_t n = static_cast<size_t>(dw) * dh;
    c->m1.assign(map1, map1 + 2 * n);
    c->m2.assign(map2, map2 + n);
    return 0;
}

int cam_start(void* hdl) {
    Cam* c = static_cast<Cam*>(hdl);
    std::lock_guard<std::mutex> lk(c->mu);
    if (c->running) return 0;
    c->pending.clear();
    if (c->cam->start(&c->cur)) return -1;
    c->running = true;
    c->ready = nullptr;
    for (auto& r : c->reqs) {
        r->reuse(Request::ReuseBuffers);
        if (c->cam->queueRequest(r.get())) return -1;
    }
    return 0;
}

int cam_stop(void* hdl) {
    Cam* c = static_cast<Cam*>(hdl);
    {
        std::lock_guard<std::mutex> lk(c->mu);
        if (!c->running) return 0;
        c->running = false;
        c->ready = nullptr;
        c->cv.notify_all();
    }
    return c->cam->stop() ? -1 : 0;   // bricht offene Requests ab (done ignoriert sie)
}

// Belichtung in µs, 0 = Automatik (Modus kurz). Wirkt ab dem nächsten eingestellten Request bzw. Start.
int cam_set_exposure(void* hdl, int us) {
    Cam* c = static_cast<Cam*>(hdl);
    std::lock_guard<std::mutex> lk(c->mu);
    c->set(controls::AeExposureMode, int32_t{1});   // kurz: ExposureShort bis 0.4, AeExposureModeShort ab 0.5, beides 1
#if XGO_LC_AB(0, 5)
    c->set(controls::ExposureTimeMode, us > 0 ? controls::ExposureTimeModeManual : controls::ExposureTimeModeAuto);
    if (us > 0) c->set(controls::ExposureTime, static_cast<int32_t>(us));
#else
    c->set(controls::ExposureTime, static_cast<int32_t>(std::max(0, us)));   // rpi: 0 = Automatik
#endif
    return 0;
}

// Analoge Verstärkung, 0 = Automatik.
int cam_set_gain(void* hdl, float gain) {
    Cam* c = static_cast<Cam*>(hdl);
    std::lock_guard<std::mutex> lk(c->mu);
#if XGO_LC_AB(0, 5)
    c->set(controls::AnalogueGainMode, gain > 0 ? controls::AnalogueGainModeManual : controls::AnalogueGainModeAuto);
    if (gain > 0) c->set(controls::AnalogueGain, gain);
#else
    c->set(controls::AnalogueGain, std::max(0.0f, gain));
#endif
    return 0;
}

// Neuestes Bild abholen: entzerrt (bzw. kopiert) direkt aus dem DMA-Puffer nach out (h Zeilen, ostride Bytes,
// BGR). Rückgabe 1 = Bild, 0 = keins innerhalb timeout_ms, -1 = Fehler.
int cam_get(void* hdl, int timeout_ms, uint8_t* out, int ostride, CamMeta* meta) {
    Cam* c = static_cast<Cam*>(hdl);
    Request* r = nullptr;
    {
        std::unique_lock<std::mutex> lk(c->mu);
        c->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [c] { return c->ready || !c->running; });
        if (!c->ready) return 0;
        r = c->ready;
        c->ready = nullptr;
    }
    int rc = 1;
    auto it = c->maps.find(r->buffers().at(c->main));
    if (it == c->maps.end()) {
        rc = -1;
    } else {
        const Mapped& m = it->second;
        const int64_t t0 = jetzt_ns();
        dma_sync(m.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
        const int64_t t1 = jetzt_ns();
        int64_t t2 = t1;
        if (!c->m1.empty()) {
            // Erst am Stück kopieren, dann aus der Kopie entzerren: der DMA-Puffer ist am Rider ungecacht eingeblendet,
            // verstreutes Lesen daraus kostete 2026-11-19 ~45 ms je Bild (ein Kern voll, 22,5 statt 24 fps).
            const size_t n = static_cast<size_t>(c->h) * c->stride;
            c->roh.resize(n);
            std::memcpy(c->roh.data(), m.p, n);
            t2 = jetzt_ns();
            if (lens_remap(c->roh.data(), c->w, c->h, c->stride, 3, c->m1.data(), c->m2.data(), out, c->w, c->h,
                           ostride)) rc = -1;
        } else {
            for (int y = 0; y < c->h; ++y)
                std::memcpy(out + static_cast<size_t>(y) * ostride, m.p + static_cast<size_t>(y) * c->stride,
                            static_cast<size_t>(c->w) * 3);
        }
        const int64_t t3 = jetzt_ns();
        dma_sync(m.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
        const int64_t t4 = jetzt_ns();
        c->t_sync += (t1 - t0) + (t4 - t3);
        c->t_kopie += t2 - t1;
        c->t_remap += t3 - t2;
        ++c->n_bilder;
    }
    const int64_t tm = jetzt_ns();
    if (meta) {
        std::memset(meta, 0, sizeof(*meta));
        meta->frame_us = -1;
        meta->exp_us = -1;
        const ControlList& md = r->metadata();
        if (auto v = md.get(controls::SensorTimestamp)) meta->ts_ns = *v;
        if (auto v = md.get(controls::FrameDuration)) meta->frame_us = *v;
        if (auto v = md.get(controls::ExposureTime)) meta->exp_us = *v;
        if (auto v = md.get(controls::AnalogueGain)) meta->gain = *v;
        if (auto v = md.get(controls::ColourGains)) {
            meta->awb[0] = (*v)[0];
            meta->awb[1] = (*v)[1];
            meta->has_awb = 1;
        }
        if (auto v = md.get(controls::ScalerCrop)) {
            meta->crop[0] = v->x;
            meta->crop[1] = v->y;
            meta->crop[2] = static_cast<int32_t>(v->width);
            meta->crop[3] = static_cast<int32_t>(v->height);
            meta->has_crop = 1;
        }
    }
    c->t_meta += jetzt_ns() - tm;
    std::lock_guard<std::mutex> lk(c->mu);
    if (meta) meta->dropped = c->dropped;
    if (c->running) c->requeue(r);
    return rc;
}

// Tatsächliche Einrichtung: [stride, raw_w, raw_h] (für das Log).
int cam_info(void* hdl, int32_t* out3) {
    Cam* c = static_cast<Cam*>(hdl);
    out3[0] = c->stride;
    out3[1] = c->raw_w;
    out3[2] = c->raw_h;
    return 0;
}

// Mittlere Zeiten je Bild seit dem letzten Aufruf in µs: [sync, kopie, remap, meta], Rückgabe Anzahl Bilder.
// Ohne Linsenmodell steht das zeilenweise Kopieren unter remap. Aus demselben Faden wie cam_get aufrufen.
int cam_zeiten(void* hdl, double* out4) {
    Cam* c = static_cast<Cam*>(hdl);
    const int64_t n = c->n_bilder;
    const double k = n > 0 ? 1e-3 / static_cast<double>(n) : 0.0;
    out4[0] = static_cast<double>(c->t_sync) * k;
    out4[1] = static_cast<double>(c->t_kopie) * k;
    out4[2] = static_cast<double>(c->t_remap) * k;
    out4[3] = static_cast<double>(c->t_meta) * k;
    c->t_sync = c->t_kopie = c->t_remap = c->t_meta = c->n_bilder = 0;
    return static_cast<int>(n);
}

void cam_close(void* hdl) { delete static_cast<Cam*>(hdl); }

}  // extern "C"

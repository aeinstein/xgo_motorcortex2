// JPEG-Kodierung im Hardware-Kodierer des Pi (bcm2835-codec-encode_image, V4L2 Memory-to-Memory, MPLANE) statt
// libjpeg auf der CPU: nur für die Anzeige im Browser (web.py). Am RiderPi kostete cv2.imencode 640x480 4,5 ms
// CPU je gesendetem Bild. Eingabe BGR3 (= OpenCV-BGR im Speicher), Ausgabe JFIF.
//
// Ein Handle = ein geöffneter Kodierer mit je einem Puffer auf beiden Seiten; nicht gleichzeitig aus mehreren
// Fäden benutzen (web.py hält eine Sperre). Braucht kein OpenCV.

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {
constexpr int kJpegAbi = 1;   // bei Änderung der Funktionen erhöhen (jpeg.py prüft das)

struct Kodierer {
    int fd = -1;
    int w = 0, h = 0, bpl = 0, quality = -1;
    void* ein = nullptr;   // Rohbild (OUTPUT-Seite)
    size_t ein_len = 0;
    void* aus = nullptr;   // JPEG (CAPTURE-Seite)
    size_t aus_len = 0;
    bool laeuft = false;
};

int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do r = ioctl(fd, req, arg); while (r == -1 && errno == EINTR);
    return r;
}

void fehler(char* err, int n, const char* was) {
    if (err && n > 0) std::snprintf(err, static_cast<size_t>(n), "%s: %s", was, std::strerror(errno));
}

int setze_qualitaet(Kodierer* k, int q) {
    if (q == k->quality) return 0;
    v4l2_control c{};
    c.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
    c.value = q;
    if (xioctl(k->fd, VIDIOC_S_CTRL, &c)) return -1;
    k->quality = q;
    return 0;
}

// je einen MMAP-Puffer anlegen und abbilden
int puffer(Kodierer* k, v4l2_buf_type typ, void** p, size_t* len) {
    v4l2_requestbuffers rb{};
    rb.count = 1;
    rb.type = typ;
    rb.memory = V4L2_MEMORY_MMAP;
    if (xioctl(k->fd, VIDIOC_REQBUFS, &rb) || rb.count < 1) return -1;
    v4l2_plane pl[VIDEO_MAX_PLANES]{};
    v4l2_buffer b{};
    b.type = typ;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = 0;
    b.m.planes = pl;
    b.length = VIDEO_MAX_PLANES;
    if (xioctl(k->fd, VIDIOC_QUERYBUF, &b)) return -1;
    *len = pl[0].length;
    *p = mmap(nullptr, *len, PROT_READ | PROT_WRITE, MAP_SHARED, k->fd, pl[0].m.mem_offset);
    return *p == MAP_FAILED ? (*p = nullptr, -1) : 0;
}

void schliesse(Kodierer* k) {
    if (k->fd >= 0 && k->laeuft) {
        int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        xioctl(k->fd, VIDIOC_STREAMOFF, &t);
        t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        xioctl(k->fd, VIDIOC_STREAMOFF, &t);
    }
    if (k->ein) munmap(k->ein, k->ein_len);
    if (k->aus) munmap(k->aus, k->aus_len);
    if (k->fd >= 0) close(k->fd);
    delete k;
}
}  // namespace

extern "C" {

int jpeg_abi(void) { return kJpegAbi; }

// Kodierer für Bilder w x h (BGR, 3 Byte je Pixel) öffnen. Rückgabe Handle oder NULL (Grund in err).
void* jpeg_open(const char* dev, int w, int h, int quality, char* err, int errlen) {
    Kodierer* k = new Kodierer;
    k->w = w;
    k->h = h;
    k->fd = open(dev, O_RDWR | O_CLOEXEC);
    if (k->fd < 0) { fehler(err, errlen, dev); schliesse(k); return nullptr; }

    v4l2_format f{};
    f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    f.fmt.pix_mp.width = static_cast<unsigned>(w);
    f.fmt.pix_mp.height = static_cast<unsigned>(h);
    f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_BGR24;
    f.fmt.pix_mp.field = V4L2_FIELD_ANY;
    f.fmt.pix_mp.num_planes = 1;
    f.fmt.pix_mp.plane_fmt[0].bytesperline = static_cast<unsigned>(w * 3);
    if (xioctl(k->fd, VIDIOC_S_FMT, &f)) { fehler(err, errlen, "S_FMT Eingabe"); schliesse(k); return nullptr; }
    if (f.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_BGR24 || static_cast<int>(f.fmt.pix_mp.width) != w ||
        static_cast<int>(f.fmt.pix_mp.height) != h) {
        errno = EINVAL;
        fehler(err, errlen, "Eingabeformat angepasst");
        schliesse(k);
        return nullptr;
    }
    k->bpl = static_cast<int>(f.fmt.pix_mp.plane_fmt[0].bytesperline);

    v4l2_format g{};
    g.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    g.fmt.pix_mp.width = static_cast<unsigned>(w);
    g.fmt.pix_mp.height = static_cast<unsigned>(h);
    g.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_JPEG;
    g.fmt.pix_mp.field = V4L2_FIELD_NONE;
    g.fmt.pix_mp.num_planes = 1;
    g.fmt.pix_mp.plane_fmt[0].sizeimage = static_cast<unsigned>(w * h * 3 / 2 + 65536);   // reicht auch bei q 100
    if (xioctl(k->fd, VIDIOC_S_FMT, &g)) { fehler(err, errlen, "S_FMT JPEG"); schliesse(k); return nullptr; }
    if (setze_qualitaet(k, quality)) { fehler(err, errlen, "Qualität"); schliesse(k); return nullptr; }

    if (puffer(k, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, &k->ein, &k->ein_len) ||
        puffer(k, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, &k->aus, &k->aus_len)) {
        fehler(err, errlen, "Puffer");
        schliesse(k);
        return nullptr;
    }
    int t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (xioctl(k->fd, VIDIOC_STREAMON, &t)) { fehler(err, errlen, "STREAMON Eingabe"); schliesse(k); return nullptr; }
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(k->fd, VIDIOC_STREAMON, &t)) { fehler(err, errlen, "STREAMON JPEG"); schliesse(k); return nullptr; }
    k->laeuft = true;
    return k;
}

// Ein Bild kodieren: bgr mit stride Bytes je Zeile (w x h wie beim Öffnen) -> out (cap Bytes).
// Rückgabe Länge des JPEG, -1 Fehler, -2 out zu klein, -3 Zeitüberschreitung (timeout_ms).
int jpeg_encode(void* hdl, const uint8_t* bgr, int stride, int quality, uint8_t* out, int cap, int timeout_ms) {
    Kodierer* k = static_cast<Kodierer*>(hdl);
    if (!k || !bgr || !out) return -1;
    if (setze_qualitaet(k, quality)) return -1;
    const size_t zeile = static_cast<size_t>(k->w) * 3;
    const size_t n_ein = static_cast<size_t>(k->bpl) * k->h;
    if (n_ein > k->ein_len) return -1;
    uint8_t* e = static_cast<uint8_t*>(k->ein);
    for (int y = 0; y < k->h; ++y)
        std::memcpy(e + static_cast<size_t>(y) * k->bpl, bgr + static_cast<size_t>(y) * stride, zeile);

    v4l2_plane pe[1]{};
    v4l2_buffer be{};
    be.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    be.memory = V4L2_MEMORY_MMAP;
    be.index = 0;
    be.m.planes = pe;
    be.length = 1;
    pe[0].bytesused = static_cast<unsigned>(n_ein);
    v4l2_plane pa[1]{};
    v4l2_buffer ba{};
    ba.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ba.memory = V4L2_MEMORY_MMAP;
    ba.index = 0;
    ba.m.planes = pa;
    ba.length = 1;
    if (xioctl(k->fd, VIDIOC_QBUF, &ba) || xioctl(k->fd, VIDIOC_QBUF, &be)) return -1;

    // beide Puffer zurückholen (auch bei Fehlern, sonst hängt der nächste Aufruf)
    int rc = 0;
    pollfd p{k->fd, POLLIN, 0};
    if (poll(&p, 1, timeout_ms) <= 0) rc = -3;
    if (rc == 0 && xioctl(k->fd, VIDIOC_DQBUF, &ba)) rc = -1;
    pollfd q{k->fd, POLLOUT, 0};
    poll(&q, 1, timeout_ms);
    xioctl(k->fd, VIDIOC_DQBUF, &be);
    if (rc) {
        // Zustand unklar: neu öffnen lassen (Aufrufer schließt bei < 0)
        return rc;
    }
    const int n = static_cast<int>(pa[0].bytesused);
    if (n <= 0) return -1;
    if (n > cap) return -2;
    std::memcpy(out, k->aus, static_cast<size_t>(n));
    return n;
}

void jpeg_close(void* hdl) {
    if (hdl) schliesse(static_cast<Kodierer*>(hdl));
}

}  // extern "C"

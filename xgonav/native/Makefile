# C++-Kern für xgonav (ctypes). Auf dem Rider bauen:  make -C xgonav/native
# Braucht g++, pkg-config und libopencv-dev in derselben Version wie python3-opencv.
# Fehlt die .so, rechnet xgonav in Python (gleiches Ergebnis, langsamer).
CXX ?= g++
CXXFLAGS ?= -O2 -mcpu=native
# -ffp-contract=off: keine FMA-Zusammenfassung, Gleitkomma wie numpy
override CXXFLAGS += -std=c++17 -fPIC -Wall -Wextra -ffp-contract=off $(shell pkg-config --cflags opencv4)
LIBS := -lopencv_video -lopencv_calib3d -lopencv_imgproc -lopencv_core

libxgofast.so: egofast.cpp gridfast.cpp navfast.cpp ortfast.cpp linsefast.cpp jpegfast.cpp floornet.o xgo.h
	$(CXX) $(CXXFLAGS) -shared -o $@ $(filter-out %.h,$^) $(LIBS)

# FloorNet: -O3 wie in der Messung (tools/floornet/bench_kern.cpp)
floornet.o: floornet.cpp floornet_kern.h
	$(CXX) $(CXXFLAGS) -O3 -c -o $@ floornet.cpp

# Kamera direkt über libcamera (statt picamera2), eigene Bibliothek: make -C xgonav/native cam
# C++20 (libcamera ab 0.8 nimmt std::span). Braucht libcamera-dev passend zur installierten libcamera (keine stabile ABI: nach apt upgrade neu bauen).
# Fehlt sie, nimmt camera.py picamera2.
cam: libxgocam.so

libxgocam.so: camfast.cpp linsefast.cpp
	$(CXX) $(CXXFLAGS) -std=c++20 $(shell pkg-config --cflags libcamera) -shared -o $@ $^ $(shell pkg-config --libs libcamera)

clean:
	rm -f libxgofast.so libxgocam.so floornet.o

.PHONY: clean cam

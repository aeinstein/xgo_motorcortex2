# C++-Kern für xgonav (ctypes). Auf dem Rider bauen:  make -C xgonav/native
# Braucht g++, pkg-config und libopencv-dev in derselben Version wie python3-opencv.
# Fehlt die .so, rechnet xgonav in Python (gleiches Ergebnis, langsamer).
CXX ?= g++
CXXFLAGS ?= -O2 -mcpu=native
# -ffp-contract=off: keine FMA-Zusammenfassung, Gleitkomma wie numpy
override CXXFLAGS += -std=c++17 -fPIC -Wall -Wextra -ffp-contract=off $(shell pkg-config --cflags opencv4)
LIBS := -lopencv_video -lopencv_calib3d -lopencv_imgproc -lopencv_core

libxgofast.so: egofast.cpp floorfast.cpp gridfast.cpp navfast.cpp floornet.o
	$(CXX) $(CXXFLAGS) -shared -o $@ $^ $(LIBS)

# FloorNet: -O3 wie in der Messung (tools/floornet/bench_kern.cpp)
floornet.o: floornet.cpp floornet_kern.h
	$(CXX) $(CXXFLAGS) -O3 -c -o $@ floornet.cpp

clean:
	rm -f libxgofast.so floornet.o

.PHONY: clean

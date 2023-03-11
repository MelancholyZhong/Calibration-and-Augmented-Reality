CC = g++
CFLAGS = -g -Wall -std=c++11 
CAM_PLIST_FLAG = -sectcreate __TEXT __info_plist Info.plist
PRODUCTS = calibrate objProjection


OPENCV = `pkg-config opencv4 --cflags --libs`

LIBS = $(OPENCV)

calibrate : calibrate.cpp helpers.cpp
	$(CC) $(CFLAGS) $(CAM_PLIST_FLAG) -o $@ $^ $(LIBS) 

objProjection : objProjection.cpp helpers.cpp
	$(CC) $(CFLAGS) $(CAM_PLIST_FLAG) -o $@ $^ $(LIBS)

.PHONY: clean

clean : 
	rm -r -f *.o *~ $(PRODUCTS) *.dSYM


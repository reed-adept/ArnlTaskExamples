
ifndef ARIA
ARIA:=/usr/local/Aria
endif

ifndef ARNL
ARNL:=/usr/local/Arnl
endif

TARGETS:=arnlServerWithAsyncTaskChain arnlServerWithTourCallbacks

CFLAGS:=-fPIC -I$(ARNL)/include -I$(ARNL)/include/Aria -I/$(ARNL)/include/ArNetworking

LFLAGS:=-L/usr/local/Arnl/lib

all: $(TARGETS)

%: %.cpp ArServerModeGoto2.cpp ArnlASyncTask.h
	$(CXX) $(CFLAGS) -o $@ $^ $(LFLAGS) -lArnl -lBaseArnl -lArNetworkingForArnl -lAriaForArnl -lpthread -ldl -lrt

clean:
	-rm $(TARGETS)

.PHONY: all clean

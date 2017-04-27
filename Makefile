
ifndef ARIA
ARIA:=/usr/local/Aria
endif

ifndef ARNL
ARNL:=/usr/local/Arnl
endif

TARGETS:=arnlServerWithAsyncTaskChain remoteArnlTaskChain

ARNL_CFLAGS:=-fPIC -I$(ARNL)/include -I$(ARNL)/include/Aria -I/$(ARNL)/include/ArNetworking
ARNL_LFLAGS:=-L$(ARNL)/lib -L$(ARNL)/lib64
ARIA_CFLAGS:=-fPIC -I$(ARIA)/include -I$(ARIA)/ArNetworking/include
ARIA_LFLAGS:=-L$(ARIA)/lib -L$(ARIA)/lib64

all: $(TARGETS)

arnlServerWithAsyncTaskChain: arnlServerWithAsyncTaskChain.cpp ArnlASyncTask.h
	$(CXX) $(ARNL_CFLAGS) -o $@ $^ $(ARNL_LFLAGS) -lArnl -lBaseArnl -lArNetworkingForArnl -lAriaForArnl -lpthread -ldl -lrt

remoteArnlTaskChain: remoteArnlTaskChain.cpp ArnlRemoteASyncTask.h
	$(CXX) $(ARIA_CFLAGS) -o $@ $^ $(ARIA_LFLAGS) -lArNetworking -lAria -lpthread -ldl -lrt

clean:
	-rm $(TARGETS)

.PHONY: all clean

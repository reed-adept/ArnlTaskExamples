
TARGETS:=arnlServerWithAsyncTaskChain arnlServerWithTourCallbacks

CFLAGS:=-fPIC -I/usr/local/Arnl/include -I/usr/local/Arnl/include/Aria -I/usr/local/Arnl/include/ArNetworking

LFLAGS:=-L/usr/local/Arnl/lib

all: $(TARGETS)

%: %.cpp ArServerModeGoto2.cpp ArnlASyncTask.h
	$(CXX) $(CFLAGS) -o $@ $^ $(LFLAGS) -lArnl -lBaseArnl -lArNetworkingForArnl -lAriaForArnl -lpthread -ldl -lrt

clean:
	-rm $(TARGETS) 

.PHONY: all clean



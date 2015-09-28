NAME	= main
CXX 	= g++

INCLUDE = -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect
CFLAGS 	= -fPIC -g -Wall `pkg-config --cflags opencv` `pkg-config --cflags libusb-1.0` `pkg-config arc --cflags --libs` -lpthread
LIBS 	= `pkg-config --libs --static opencv` `pkg-config --libs libusb-1.0` `pkg-config arc --cflags --libs` -lpthread

all: main

main:./bin/main.o ./bin/raspberry.o
	$(CXX) ./bin/main.o ./bin/raspberry.o -o $@  $(LIBS) $(FREE_LIBS)

./bin/main.o:./src/main.cpp
	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

./bin/raspberry.o:../../lib/raspberry.cpp
	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

clean:
	rm -rf ./bin/* ./main

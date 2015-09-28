NAME 	= main
SUB		= camera
CXX 	= g++

OUT		= $(NAME)

INCLUDE = -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect
CFLAGS 	= -std=c++11 -fPIC `pkg-config --cflags opencv` `pkg-config --cflags libusb-1.0` `pkg-config arc --cflags --libs` -lpthread
LIBS 	= -std=c++11 `pkg-config --libs --static opencv` `pkg-config --libs libusb-1.0` `pkg-config arc --cflags --libs` -lpthread

all: main

main:./bin/main.o ./bin/$(SUB).o
	$(CXX) ./bin/main.o ./bin/$(SUB).o -o $@  $(LIBS) $(FREE_LIBS)

./bin/main.o:./src/main.cpp
	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

./bin/$(SUB).o:../../lib/$(SUB).cpp
	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

clean:
	rm -rf ./bin/* ./main

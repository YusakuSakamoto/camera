NAME 	= main
SUB		= tomato
CXX 	= g++

OUT		= $(NAME)

INCLUDE = -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect
CFLAGS 	= -fPIC -g -Wall `pkg-config --cflags opencv` `pkg-config --cflags libusb-1.0`
LIBS 	= `pkg-config --libs --static opencv` `pkg-config --libs libusb-1.0`

all: $(OUT)

$(OUT):./bin/main.o ./bin/tomato.o
	$(CXX) ./bin/main.o ./bin/tomato.o -o $@  $(LIBS) $(FREE_LIBS)

./bin/main.o:./src/main.cpp
	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

./bin/tomato.o:./src/tomato.cpp
	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

clean:
	rm -rf ./bin/*.o $(OUT)

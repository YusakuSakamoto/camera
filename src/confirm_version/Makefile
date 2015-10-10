NAME 	= main
CXX 	= g++

OUT		= $(NAME)

INCLUDE = -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect
CFLAGS 	= -fPIC -g -Wall `pkg-config --cflags opencv` `pkg-config --cflags libusb-1.0`
LIBS 	= `pkg-config --libs --static opencv` `pkg-config --libs libusb-1.0`

all: $(OUT)

$(OUT):main.cpp
	$(CXX) $< $(LIBS) $(FREE_LIBS) $(INCLUDE) $(CFLAGS) -o $@

#main.o:main.cpp
#	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

clean:
	rm -rf ./bin/*.o $(OUT)

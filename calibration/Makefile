NAME	= main
CXX 	= g++

OUT		= $(NAME:%=%.out)
BIN		= bin/$(NAME:%=%.o)
SRC		= src/$(NAME:%=%.cpp)

INCLUDE = -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect
CFLAGS 	= -fPIC -g -Wall `pkg-config --cflags opencv` `pkg-config --cflags libusb-1.0` `pkg-config sakamoto --cflags --libs` -lpthread
LIBS 	= `pkg-config --libs --static opencv` `pkg-config --libs libusb-1.0` `pkg-config sakamoto --cflags --libs` -lpthread

.PHONY:install

all: $(OUT)

$(OUT):./bin/calib.o ./bin/main.o
	$(CXX) ./bin/calib.o ./bin/main.o -o $@  $(LIBS) $(FREE_LIBS)

./bin/main.o: ./src/main.cpp
	$(CXX) -c ./src/main.cpp $(INCLUDE) $(LIBS) $(CFLAGS) -o ./bin/main.o

./bin/calib.o:./src/calib.cpp
	$(CXX) -c ./src/calib.cpp $(INCLUDE) $(LIBS) $(CFRAGS) -o ./bin/calib.o

clean:
	rm -rf ./bin/calib.o ./bin.main.o ./main.out

install:
	cd install && make && sh install.sh

remove:
	cd install && make clean && cd ../sample && make clean

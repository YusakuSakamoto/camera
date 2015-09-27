NAME	= opencv2
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

$(OUT): $(BIN)
	$(CXX) $< -o $@  $(LIBS) $(FREE_LIBS)

$(BIN): $(SRC)
	$(CXX) -c $< $(INCLUDE) $(CFLAGS) -o $@

clean:
	rm -rf $(BIN) ./*.out

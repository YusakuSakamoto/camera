CC	=g++
TARGET = main
OBJ = $(TARGET:%.cpp=%)
CFLAGS	=`pkg-config arc --cflags --libs`

.PHONY:clean
all:./src/$(TARGET).cpp
	$(CC) $< $(FLAGS) $(CFLAGS) -o $(TARGET)

clean:
	rm $(TARGET)

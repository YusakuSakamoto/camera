CC	=g++
TARGET = sample
OBJ = $(TARGET:%.cpp=%)
CFLAGS	=`pkg-config sakamoto --cflags --libs`

.PHONY:clean
all:./src/$(TARGET).cpp
	$(CC) $< $(FLAGS) $(CFLAGS) -o $(TARGET)

clean:
	rm $(TARGET)

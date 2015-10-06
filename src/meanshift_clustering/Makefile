LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui
CC = g++

all: ./bin/main.o ./bin/MeanShift.o
	$(CC) ./bin/main.o ./bin/MeanShift.o -o main $(LIBS)
./bin/main.o: ./src/main.cpp
	$(CC) -c ./src/main.cpp $(LIBS) -o $@
./bin/MeanShift.o: ./src/MeanShift.cpp
	$(CC) -c ./src/MeanShift.cpp $(LIBS) -o $@
clean:
	rm ./bin/*.o main

.PHONY: all clean

all: example

clean:
	rm -vf example

example: ina219.hpp ina219.cpp example.cpp
	g++ -O1 -g -flto ina219.cpp example.cpp -o example

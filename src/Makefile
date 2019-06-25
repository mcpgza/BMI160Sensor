CXXFLAGS=-Wall -O3 -g -Wextra -Wno-unused-parameter
OBJECTS=main.o bmi160.o
BINARIES=main


all : $(BINARIES)

main: main.o bmi160.o
	$(CXX) $(CXXFLAGS) main.o bmi160.o -o $@

%.o : %.cc
	$(CXX) -I$(RGB_INCDIR) $(CXXFLAGS) -c -o $@ $<

main.o : main.cpp
	$(CXX) -I$(RGB_INCDIR) $(CXXFLAGS) $(MAGICK_CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECTS) $(BINARIES)

FORCE:
.PHONY: FORCE

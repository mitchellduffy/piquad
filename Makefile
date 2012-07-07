SOURCES=$(wildcard src/*.cpp)
OBJECTS := $(addprefix obj/,$(notdir $(SOURCES:.cpp=.o)))
TOOLSRC:=$(wildcard tools/*.cpp)
TOOLS:=$(TOOLSRC:.cpp=.out)

LD_FLAGS := -lbcm2835 -lboost_thread
EXECUTABLE=fenpi

all: $(SOURCES) $(OBJECTS) $(TOOLS) $(EXECUTABLE)


$(EXECUTABLE): $(OBJECTS) 
	cp tools/main.out ./fenpi
obj/%.o: src/%.cpp
	g++ -c -O -o $@ $<
tools/%.out: tools/%.cpp
	g++ -O -o $@ $< $(LD_FLAGS) $(OBJECTS) -Isrc
clean:
	rm -f fenpi
	rm -f obj/*.o
	rm -f tools/*.out


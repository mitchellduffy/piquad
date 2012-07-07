SOURCES=$(wildcard *.cpp)
OBJECTS := $(SOURCES:.cpp=.o)

CC_FLAGS := 
EXECUTABLE=pwm

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	g++ -o $@ -O $(OBJECTS) -lboost_thread -lbcm2835

%.o: %.cpp
	g++ $(CC_FLAGS) -c -o $@ $< 
clean:
	rm *.o


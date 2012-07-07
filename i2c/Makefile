SOURCES=main.cpp L3G4200D.cpp  LSM303.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=a.out

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	g++ $(OBJECTS) -o $@ -O

.cpp.o: 
	g++ -c $< -o $@ -O
clean:
	rm *.o

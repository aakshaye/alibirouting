#Phony target.
current_target: main_gc gc_distance


#Compiler flags
#CXXFLAGS = -O0 -Wall -ggdb
CXXFLAGS = -O2 -fPIC $(DFLAGS)
#CXXFLAGS = -O2 -fPIC
#Linker flags
#LDFLAGS = -O0 -Wall -ggdb
LDFLAGS = -O2

#Modules
gc_distance.o: gc_distance.cpp gc_distance.hpp
main_gc.o: main_gc.cpp gc_distance.hpp
gc_distance_wrap.o: gc_distance.i gc_distance.hpp
	swig -c++ -classic -python -py3 gc_distance.i
	g++ $(CXXFLAGS) -c gc_distance_wrap.cxx $(shell python3-config --includes)

#Python Module
gc_distance: gc_distance.o gc_distance_wrap.o
	g++ -shared gc_distance.o gc_distance_wrap.o -o _gc_distance.so


#Executables
main_gc: gc_distance.o main_gc.o
	g++ -o main_gc $(CXXFLAGS) gc_distance.o main_gc.o


clean:   
	rm -f gc_distance_wrap.cxx gc_distance.py gc_distance.pyc _gc_distance.so
	rm -f main_gc *.o 

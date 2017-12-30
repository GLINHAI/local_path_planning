PROJ = ReadDemo
OBJ = read.o
INCLUDE_DIR = /usr/include/eigen3

CXX = g++ -std=c++11
CXXFLAGS = -I $(INCLUDE_DIR)

$(PROJ) : $(OBJ)
	$(CXX) $^ -o $@ 

clean:
	$(RM) *.o
	$(RM) $(PROJ)

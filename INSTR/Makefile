# won't run on windows

# Compiler settings
# automates the g++ and other compiler commands for all necessary files -- ensures all files are properly linked together
CXX = g++
CXXFLAGS = -O2 -march=native -g -Wall -Wextra -std=c++17 -fopenmp -Iinclude -I/usr/local/include/opencv4 -I/home/scout/vimba-x/VimbaX_2026-2/api/include -I/home/scout/vimba-x/VimbaX_2026-2/api/include/VmbCPP
# all -I point to folders with .hpp files

# OpenCV configuration 
# points to all library dependencies that each system have
OPENCV = $(shell pkg-config --cflags --libs opencv4)

# Alvium camera configuration
ALVIUM = -L/home/scout/vimba-x/VimbaX_2026-2/api/lib -Wl,-rpath,/home/scout/vimba-x/VimbaX_2026-2/api/lib -lVmbCPP -lVmbC

# set directory for vimba/alvium
`DVMB_DIR=/home/scout/vimba-x/VimbaX*`

# Library source files -- defines more variables that are used in primary compilation command
SRC = $(wildcard src/*.cpp) # grabs all .cpp files in src folder
#SRC = $(filter-out $(wildcard src/Arduino*.cpp), $(wildcard src/*.cpp))
OBJ = $(SRC:.cpp=.o) # makes object files for all .cpp files in source i.e all .o files which it uses to compile

# Main application
MAIN = main.cpp
MAIN_OBJ = main.o
TARGET = bin/main # -o output file 

# Test files
TEST_SRC = $(wildcard testing/*.cpp) # grabs all testing files .cpp
#TEST_SRC = $(filter-out $(wildcard src/Arduino*.cpp), $(wildcard testing/*.cpp))
TESTS = $(patsubst testing/%.cpp,bin/%,$(TEST_SRC)) # makes sure that all files compiled in testing are in the bin folder

# Primary programs - all .cpp files in pwd (excluding main.cpp)
PROG_SRC = $(filter-out $(MAIN), $(wildcard *.cpp)) # all the .cpp files in INSTR except for main.cpp
PROGS = $(patsubst %.cpp,bin/%,$(PROG_SRC))

# Default target -- if you just run "make" without anything else it calls directories below which runs the code listed below
all: directories $(TARGET) tests programs
# Note: $(X) accesses the value of the given variable rather than setting it

# Build main executable
$(TARGET): $(MAIN_OBJ) $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(ALVIUM) $(OPENCV)

# Explicitly compile main.cpp to main.o
$(MAIN_OBJ): $(MAIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Build all tests
tests: $(TESTS)

# Build all primary programs
programs: $(PROGS)

# Build individual test executables
bin/%: testing/%.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(ALVIUM) $(OPENCV) # builds/loads everything defined before to build the overall compialation command 

# Build individual primary program executables
bin/%: %.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(ALVIUM) $(OPENCV)

# Compile source files into object files
src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


# Create output directory
directories: # for linux
	mkdir -p bin


# Remove generated files -- deletes all old object files in the bin folder when running "make clean"
clean:
# for linux
	rm -f src/*.o *.o
	rm -rf bin
# for windows
#del /q /f src\*.o *.o


.PHONY: all clean tests directories programs

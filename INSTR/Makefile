# Compiler settings
CXX = g++
CXXFLAGS = -g -Wall -Wextra -std=c++17 -fopenmp -Iinclude -I/usr/include/opencv4

# OpenCV configuration 
OPENCV = `pkg-config --cflags --libs opencv4`

# Library source files
SRC = $(wildcard src/*.cpp)
OBJ = $(SRC:.cpp=.o)

# Main application
MAIN = main.cpp
MAIN_OBJ = main.o
TARGET = bin/main

# Test files
TEST_SRC = $(wildcard testing/*.cpp)
TESTS = $(patsubst testing/%.cpp,bin/%,$(TEST_SRC))

# Primary programs - all .cpp files in pwd (excluding main.cpp)
PROG_SRC = $(filter-out $(MAIN), $(wildcard *.cpp))
PROGS = $(patsubst %.cpp,bin/%,$(PROG_SRC))

# Default target
all: directories $(TARGET) tests programs

# Build main executable
$(TARGET): $(MAIN_OBJ) $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV)

# Explicitly compile main.cpp to main.o
$(MAIN_OBJ): $(MAIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Build all tests
tests: $(TESTS)

# Build all primary programs
programs: $(PROGS)

# Build individual test executables
bin/%: testing/%.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV)

# Build individual primary program executables
bin/%: %.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV)

# Compile source files into object files
src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


# Create output directory
directories: # for linux
	mkdir -p bin


# Remove generated files
clean:
# for linux
	rm -f src/*.o *.o
# for windows
# del /q /f src\*.o *.o


.PHONY: all clean tests directories programs

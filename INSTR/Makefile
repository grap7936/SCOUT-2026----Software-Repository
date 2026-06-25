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


# Default target
all: directories $(TARGET) tests


# Build main executable (Fixed linking order and main compilation)
$(TARGET): $(MAIN_OBJ) $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV)

# Explicitly compile main.cpp to main.o
$(MAIN_OBJ): $(MAIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@


# Build all tests
tests: $(TESTS)


# Build individual test executables (Fixed linking order)
bin/%: testing/%.cpp $(OBJ)
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
	rm -f src/*.o main.o
# for windows
# del /q /f src\*.o main.o


.PHONY: all clean tests directories

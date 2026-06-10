# Compiler settings
CXX = g++
CXXFLAGS = -g -Wall -Wextra -std=c++17 -Iinclude -IC:/msys64/ucrt64/include/opencv4

# OpenCV configuration
OPENCV = $(shell pkg-config --cflags --libs opencv4)

# Library source files
SRC = $(wildcard src/*.cpp)
OBJ = $(SRC:.cpp=.o)

# Main application
MAIN = main.cpp
TARGET = bin/main

# Test files
TEST_SRC = $(wildcard testing/*.cpp)
TESTS = $(patsubst testing/%.cpp,bin/%,$(TEST_SRC))


# Default target
all: directories $(TARGET) tests


# Build main executable
$(TARGET): $(MAIN) $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(OPENCV)


# Build all tests
tests: $(TESTS)


# Build individual test executables
bin/%: testing/%.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) $< $(OBJ) -o $@ $(OPENCV)


# Compile source files into object files
src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


# Create output directory
directories:
	mkdir -p bin


# Remove generated files
clean:
# for linux
# rm -f src/*.o
# for windows
	del /q /f src\*.o


.PHONY: all clean tests directories
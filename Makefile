CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g -fdiagnostics-color=always


SRCS = main.cpp optdev.cpp mirror.cpp ray.cpp lenses.cpp mpvector.cpp mpio.cpp constants.cpp config_loader.cpp geometry_loader.cpp
OBJS = $(SRCS:.cpp=.o)
EXEC = main

all: run clean

# Link object files into executable
$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile .cpp files into .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJS)

# Run the program
run: $(EXEC)
	./$(EXEC)

.PHONY: all clean run
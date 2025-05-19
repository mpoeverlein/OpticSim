CXX = g++
CC = gcc

# Flags for C++ files
CXXFLAGS = -std=c++20 -Wall -Wextra -g -fdiagnostics-color=always
# Flags for C files
CFLAGS = -Wall -Wextra -g

# Libraries and includes
LIBS = -lglfw -lm
INC = -I/Users/max/Programs/glfw-3.4/include \
      -I/Users/max/Programs/glad/include \
      -I/Users/max/Programs/OpticSim/glad/include \
      -I/usr/local/Cellar/glm/1.0.1/include
LIB_DIRS = -L/Users/max/Programs/glfw-3.4/build/src

# Source files
C_SRCS = glad/src/glad.c
CPP_SRCS = main.cpp optdev.cpp mirror.cpp ray.cpp lenses.cpp mpvector.cpp mpio.cpp constants.cpp config_loader.cpp geometry_loader.cpp visualizeglfw.cpp material.cpp
OBJS = $(CPP_SRCS:.cpp=.o) $(C_SRCS:.c=.o)
# OBJS := $(addprefix $(OBJDIR)/,$(notdir $(OBJS)))

EXEC = main

all: $(EXEC)

# Link all object files
$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $(LIB_DIRS) -o $@ $^ $(LIBS)

# Compile C++ files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INC) -c $< -o $@

# Compile C files
%.o: %.c
	$(CC) $(CFLAGS) $(INC) -c $< -o $@

clean:
	rm -f $(OBJS) $(EXEC)

run: $(EXEC)
	./$(EXEC)

.PHONY: all clean run
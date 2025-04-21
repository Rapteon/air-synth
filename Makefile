CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I.
LDFLAGS = -lrtaudio -lstk -lgpiod -lpthread

# STK (Synthesis ToolKit) installation directory
# Modify these paths to match your STK installation
STK_DIR = /usr/local/include/stk
STK_LIB = /usr/local/lib

# Main target
TARGET = mpu6050

# Source files and their corresponding header files
SRCS = Synth/Synth.cpp mpu6050.cpp ControllerEvent/ControllerEvent.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Include directories
INCLUDES = -I$(STK_DIR) -I.

# Main rule
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ -L$(STK_LIB) $(LDFLAGS)

# Compilation
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Create directory structure
setup:
	mkdir -p Synth

# Clean
clean:
	rm -f $(TARGET) $(OBJS)

# Install dependencies (For Debian/Ubuntu)
deps:
	sudo apt-get update
	sudo apt-get install -y g++ librtaudio-dev libgpiod-dev libstk0-dev

.PHONY: all clean setup deps
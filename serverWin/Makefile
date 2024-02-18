# Makefile for server.cpp

# Compiler and compiler flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall

# Linker flags
LDFLAGS = -lws2_32 -lpthread

# Executable name
SERVER_EXECUTABLE = server.exe
CLIENT_EXECUTABLE = client.exe

# Source files
SERVER_SOURCES = server.cpp
CLIENT_SOURCES = client.cpp

# Object files
SERVER_OBJECTS = $(SERVER_SOURCES:.cpp=.o)
CLIENT_OBJECTS = $(CLIENT_SOURCES:.cpp=.o)

# Default target
.PHONY: all 
all: clean $(SERVER_EXECUTABLE) $(CLIENT_EXECUTABLE) clean_obj

# Linking the executables
$(SERVER_EXECUTABLE): $(SERVER_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(CLIENT_EXECUTABLE): $(CLIENT_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compiling source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	del $(SERVER_EXECUTABLE) $(CLIENT_EXECUTABLE) $(SERVER_OBJECTS) $(CLIENT_OBJECTS)

.PHONY: clean_obj
clean_obj:
# Use "del" on Windows and "rm" on Unix-like systems
	del $(SERVER_OBJECTS) $(CLIENT_OBJECTS)


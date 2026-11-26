CXX := g++

TARGET := runsim
SRC_DIR := src
OBJ_DIR := objects
EIGEN_DIR ?= /usr/include/eigen3

CPPFLAGS += -Iinclude -I$(EIGEN_DIR)
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic

SOURCES := $(shell find $(SRC_DIR) -type f -name '*.cpp')
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
DEPS := $(OBJECTS:.o=.d)

.PHONY: all clean
.DEFAULT_GOAL := all

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

clean:
	$(RM) $(OBJECTS) $(DEPS) $(TARGET)

-include $(DEPS)

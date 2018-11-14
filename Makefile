TARGET=program
CXX=g++
CXXFLAGS=`pkg-config --cflags tesseract opencv` -std=c++11
LDLIBS=`pkg-config --libs tesseract opencv`
BUILD    := ./build
OBJ_DIR  := $(BUILD)/objects
APP_DIR  := $(BUILD)/apps
INCLUDE  := -I include/
SRCS:=$(wildcard *.cpp) \
	  $(wildcard src/*.cpp)
OBJS := $(SRCS:%.cpp=$(OBJ_DIR)/%.o)

all: build $(APP_DIR)/$(TARGET) 

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<

$(APP_DIR)/$(TARGET): $(OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(LDLIBS) -o $(APP_DIR)/$(TARGET) $(OBJS)

.PHONY: all build clean debug release

build:
	@mkdir -p $(APP_DIR)
	@mkdir -p $(OBJ_DIR)
	
debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O2
release: all

clean:
	-@rm -rvf $(OBJ_DIR)/*
	-@rm -rvf $(APP_DIR)/*
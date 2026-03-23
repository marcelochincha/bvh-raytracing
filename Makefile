# ====== CONFIG ======
CXX := C:/mingw64/bin/g++.exe

SRC_DIR := src
BUILD_DIR := build
BIN_DIR := bin

TARGET := app.exe

INCLUDES := -Ithird_party -Isrc
LIBS := -Llib -lmingw32 -lSDL2main -lSDL2 -lSDL2_mixer

CXXFLAGS := -O1 -g -m64 -march=native -fopt-info-vec-optimized
# Recursive wildcard to pick up sources in nested folders
rwildcard = $(wildcard $1/$2) $(foreach d,$(wildcard $1/*),$(call rwildcard,$d,$2))
# ====================

SRCS := $(call rwildcard,$(SRC_DIR),*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

all: $(BIN_DIR)/$(TARGET)

$(BIN_DIR)/$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(OBJS) $(LIBS) -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $< $(INCLUDES) $(CXXFLAGS) -o $@

run: all
	./$(BIN_DIR)/$(TARGET) $(ARGS)

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)/$(TARGET)

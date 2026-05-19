# ====== CONFIG ======
CXX := C:/mingw64/bin/g++.exe

SRC_DIR := src
BUILD_DIR := build
BIN_DIR := bin

TARGET := app.exe

INCLUDES := -Ithird_party -Isrc
LIBS := -Llib -lmingw32 -lSDL2main -lSDL2 -lSDL2_mixer -lpsapi

CXXFLAGS := -O1 -g -m64 -march=native -fopt-info-vec-optimized -MMD -MP
# Recursive wildcard to pick up sources in nested folders
rwildcard = $(wildcard $1/$2) $(foreach d,$(wildcard $1/*),$(call rwildcard,$d,$2))
# ====================

# ====== RUN PARAMS ======
# Override any of these: make run WIDTH=640 HEIGHT=400 FPS=30 DEBUG=1
WIDTH      ?= 814
HEIGHT     ?= 480
FPS        ?= 60
AUDIO_RATE ?= 
DEBUG      ?= 

RUN_ARGS :=
ifneq ($(WIDTH),)
  RUN_ARGS += --width $(WIDTH)
endif
ifneq ($(HEIGHT),)
  RUN_ARGS += --height $(HEIGHT)
endif
ifneq ($(FPS),)
  RUN_ARGS += --fps $(FPS)
endif
ifneq ($(AUDIO_RATE),)
  RUN_ARGS += --audio-rate $(AUDIO_RATE)
endif
ifneq ($(DEBUG),)
  RUN_ARGS += --debug
endif
RUN_ARGS += $(ARGS)
# ========================

SRCS := $(call rwildcard,$(SRC_DIR),*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))
OBJS_REL := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/rel/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)
-include $(DEPS)

all: $(BIN_DIR)/$(TARGET)

$(BIN_DIR)/$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(OBJS) $(LIBS) -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $< $(INCLUDES) $(CXXFLAGS) -o $@

CXXFLAGS_REL := -O2 -m64 -march=native -DNDEBUG -MMD -MP

release: $(BIN_DIR)/$(TARGET)_rel.exe
	strip $(BIN_DIR)/$(TARGET)_rel.exe
	@echo "Release build: $(BIN_DIR)/$(TARGET)_rel.exe"

$(BIN_DIR)/$(TARGET)_rel.exe: $(OBJS_REL)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(OBJS_REL) $(LIBS) -o $@

$(BUILD_DIR)/rel/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -c $< $(INCLUDES) $(CXXFLAGS_REL) -o $@

run: all
	./$(BIN_DIR)/$(TARGET) $(RUN_ARGS)

run-hd: all
	./$(BIN_DIR)/$(TARGET) --width 880 --height 508 --fps 90

run-small: all
	./$(BIN_DIR)/$(TARGET) --width 320 --height 200 --fps 60

run-debug: all
	./$(BIN_DIR)/$(TARGET) --debug $(RUN_ARGS)

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)/$(TARGET)

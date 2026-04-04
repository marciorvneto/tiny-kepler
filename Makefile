OUT_DIR := ./out
CFLAGS := -g
CPPFLAGS := -I. -I./vendor/tinyla
LDFLAGS := -lm

RAYLIB_DIR = vendor/raylib/src
RAYLIB_LIB = $(RAYLIB_DIR)/libraylib.a
RAYLIB_FLAGS = -I$(RAYLIB_DIR) -L$(RAYLIB_DIR) -lraylib -lpthread -ldl -lrt -lX11

EXAMPLES := $(filter-out examples/orbit-viewer.c, $(wildcard examples/*.c))
EXAMPLES_BINS := $(patsubst examples/%.c, $(OUT_DIR)/%, $(EXAMPLES))

all: $(OUT_DIR)/main $(EXAMPLES_BINS)

$(RAYLIB_LIB):
	$(MAKE) -C $(RAYLIB_DIR) PLATFORM=PLATFORM_DESKTOP

$(OUT_DIR)/main: main.c ./tiny-kepler.h | $(OUT_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LDFLAGS)

$(OUT_DIR)/orbit-viewer: examples/orbit-viewer.c ./tiny-kepler.h $(RAYLIB_LIB) | $(OUT_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -I$(RAYLIB_DIR) -o $@ $< $(RAYLIB_LIB) $(LDFLAGS) -lpthread -ldl -lrt -lX11

$(OUT_DIR)/%: examples/%.c ./tiny-kepler.h | $(OUT_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< $(LDFLAGS)

$(OUT_DIR):
	@mkdir -p $(OUT_DIR)

.PHONY: all clean viewer
viewer: $(OUT_DIR)/orbit-viewer

clean:
	@find . -path "$(OUT_DIR)/*" -not -name "*.py" -delete

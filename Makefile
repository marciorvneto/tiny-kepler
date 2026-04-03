OUT_DIR := ./out

CFLAGS := -g
CPPFLAGS := -I. -I./vendor/tinyla
LDFLAGS := -lm

EXAMPLES := $(wildcard examples/*.c)
EXAMPLES_BINS := $(patsubst examples/%.c, $(OUT_DIR)/%, $(EXAMPLES))

all: $(OUT_DIR)/main $(EXAMPLES_BINS)

$(OUT_DIR)/main: main.c ./tiny-kepler.h| $(OUT_DIR) 
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< ${LDFLAGS}

$(OUT_DIR)/%: examples/%.c ./tiny-kepler.h | $(OUT_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< ${LDFLAGS}



$(OUT_DIR):
	@mkdir -p $(OUT_DIR)

.PHONY: all clean

clean:
		@find . -path "$(OUT_DIR)/*" -not -name "*.py" -delete


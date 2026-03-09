OUT_DIR := ./out

C_FLAGS := -g

$(OUT_DIR)/main: main.c | $(OUT_DIR)
	$(CC) $(C_FLAGS) -o $@ $<

all: main

$(OUT_DIR):
	@mkdir -p $(OUT_DIR)


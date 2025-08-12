CC = gcc
CFLAGS = -std=c89 -Wall -Wextra -Werror -pedantic

SRC = $(wildcard *.c)
BIN_DIR = bin
EXE = $(patsubst %.c,$(BIN_DIR)/%,$(SRC))

.PHONY: all run clean list

all: $(BIN_DIR) $(EXE)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BIN_DIR)/%: %.c
	$(CC) $(CFLAGS) -o $@ $<

run: all
	@echo "Running all C89 examples...";
	@set -e; for f in $(EXE); do \
	  echo "==> $$f"; \
	  "$$f" | sed 's/^/  /'; \
	  echo; \
	done

list:
	@printf "Executables to build:\n"; printf "  %s\n" $(EXE)

clean:
	rm -rf $(BIN_DIR)




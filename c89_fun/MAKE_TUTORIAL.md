## Make Tutorial (Practical)

This is a quick, pragmatic guide tailored to this repo.

### Core Concepts

- Targets: names you can `make`, e.g., `all`, `run`, `clean`.
- Rules: how to build a target from prerequisites.
- Variables: like `CC`, `CFLAGS`, usable throughout the Makefile.
- Pattern rules: `%` matches any stem, e.g., `bin/%: %.c`.

### Variables in this project

- `CC = gcc`: the C compiler.
- `CFLAGS = -std=c89 -Wall -Wextra -Werror -pedantic`: strict compilation flags.
- `SRC = $(wildcard *.c)`: all `.c` files in this directory.
- `BIN_DIR = bin`: output directory for built executables.
- `EXE = $(patsubst %.c,$(BIN_DIR)/%,$(SRC))`: map `x.c` → `bin/x`.

### Important Targets

- `all`: ensures `bin/` exists, then builds every `$(EXE)`.
- `run`: depends on `all`; runs every executable in order, printing headers.
- `list`: shows which executables will be built.
- `clean`: deletes `bin/`.

### Pattern Rule Explained

```
$(BIN_DIR)/%: %.c
	$(CC) $(CFLAGS) -o $@ $<
```

- `$@` is the target (e.g., `bin/foo`).
- `$<` is the first prerequisite (e.g., `foo.c`).
- This compiles any `.c` into a matching binary under `bin/`.

### Tips

- Rebuild only what changed: Make calculates which files are out-of-date.
- Phony targets: mark non-file targets with `.PHONY` to avoid filename clashes.
- Parallel builds: `make -j` to compile faster.
- Verbose builds: by default commands are shown; you can prefix lines with `@` to silence.

### Extending

- To add a new C file `foo.c`, just drop it in `c89_fun/` and rerun `make`. It will be compiled to `bin/foo` automatically.
- To exclude a file from the default build, consider renaming it with a suffix and adjusting `SRC` to filter (e.g., use a pattern or a curated list).

### Debugging Makefiles

- `make -n`: dry-run, prints commands without executing.
- `make -d`: very verbose debug output.
- `make VAR=value`: override variables at invocation, e.g., `make CFLAGS=-O2`.



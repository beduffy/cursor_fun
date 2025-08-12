CC = gcc
CFLAGS = -std=c89 -Wall -Wextra -Werror -pedantic

# Curated sources (exclude http_get_posix.c from default)
SRC = \
 basics_hello.c \
 bitwise_operations.c \
 time_rand_errno.c \
 structs_unions_enums.c \
 setjmp_longjmp.c \
 file_io.c \
 arrays_and_pointers.c \
 memory_management.c \
 control_flow.c \
 types_and_limits.c \
 strings_basics.c \
 headers_and_linkage.c \
 varargs_example.c \
 const_volatile.c \
 function_pointers.c \
 signals_example.c \
 macros_preprocessor.c \
 typedef_and_bitfields.c \
 sequence_points_notes.c \
 qsort_bsearch.c \
 floating_point.c \
 alignment_and_padding.c \
 endianness.c \
 memory_pool.c \
 intrusive_list.c \
 hash_table_linear_probing.c \
 dynamic_array.c \
 fsm_state_machine.c \
 test_dynamic_array.c \
 bitset.c \
 crc32.c \
 utf8_validate.c \
 ring_buffer.c \
 base64.c \
 base64_decode.c \
 bst_tree.c \
 json_tokenizer.c \
 json_ast_parser.c \
 json_writer.c \
 json_query.c \
 json_pretty.c \
 mini_printf.c \
 kmp_search.c \
 arena_allocator.c \
 arena_scratch.c \
 priority_scheduler.c \
 ini_writer.c \
 ini_parser.c \
 raster_ppm.c \
 png_writer.c \
 wav_synth.c \
 string_intern.c \
 string_hashmap.c \
 lz77_tiny.c \
 lz4_lite.c \
 donut_ascii.c \
 union_find.c \
 fixed_point.c
BIN_DIR = bin
EXE = $(patsubst %.c,$(BIN_DIR)/%,$(SRC))

.PHONY: all run clean list

all: $(BIN_DIR) $(EXE)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BIN_DIR)/%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS_$*) $(LDLIBS_$*)

# Per-program libs
LDLIBS_wav_synth = -lm

run: all
	@echo "Running C89 examples...";
	@set -e; for f in $(EXE); do \
	  name=$$(basename $$f); \
	  if [ -z "$(FILTER)" ] || echo "$$name" | grep -E "$(FILTER)" >/dev/null 2>&1; then \
	    echo "==> $$f"; \
	    "$$f" | sed 's/^/  /'; \
	    echo; \
	  fi; \
	done

list:
	@printf "Executables to build:\n"; printf "  %s\n" $(EXE)

clean:
	rm -rf $(BIN_DIR)




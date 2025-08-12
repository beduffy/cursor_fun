## C89 Educational Suite

This folder contains many small, focused C89 programs. Each file is a self-contained lesson that builds intuition about C89 and its sharp edges. Everything compiles with strict flags: `-std=c89 -Wall -Wextra -Werror -pedantic`.

### Build and Run

- Build all examples:
  ```bash
  make -C c89_fun all
  ```
- Run all examples sequentially:
  ```bash
  make -C c89_fun run
  ```
- Clean binaries:
  ```bash
  make -C c89_fun clean
  ```

Notes:
- `strings_basics.c` has an optional interactive input demo that is disabled during `make run` to avoid blocking. Enable it by running with `C89_INTERACTIVE=1`:
  ```bash
  C89_INTERACTIVE=1 ./c89_fun/bin/strings_basics
  ```

### File Guide (What each file teaches)

- `basics_hello.c`: printing, basic types, simple arithmetic.
- `types_and_limits.c`: integer and floating limits from `<limits.h>` and `<float.h>`.
- `control_flow.c`: `if/else`, `switch`, `for`, `while`, `do-while`.
- `arrays_and_pointers.c`: arrays, decay to pointers, pointer arithmetic, addresses.
- `strings_basics.c`: C strings, `strlen`, `strcpy` caveats, safe input via `fgets`.
- `structs_unions_enums.c`: aggregate types, memory view with `union`, and enums.
- `typedef_and_bitfields.c`: `typedef` and compact bitfields (layout is implementation-defined).
- `macros_preprocessor.c`: macros, stringizing with `#`, `__FILE__` and line stringization.
- `mymath.h` + `headers_and_linkage.c`: include guards, prototypes, `static` internal linkage.
- `const_volatile.c`: `const` correctness and a `volatile` example.
- `function_pointers.c`: function pointers and passing behavior.
- `varargs_example.c`: variable arguments with `<stdarg.h>`.
- `file_io.c`: basic file I/O with `fopen`, `fprintf`, `fgets`, `fclose`.
- `memory_management.c`: `malloc`, `realloc`, `free`, error checks.
- `qsort_bsearch.c`: `qsort`/`bsearch` with a compare function for `int`.
- `time_rand_errno.c`: `time`, `rand`, and demonstrating `errno` usage.
- `signals_example.c`: basic signals with `SIGINT` and `signal`/`raise`.
- `setjmp_longjmp.c`: non-local jumps with `setjmp`/`longjmp`.
- `bitwise_operations.c`: masks, shifts, and bitwise operators.
- `floating_point.c`: precision, `DBL_EPSILON`, approximate comparisons.
- `sequence_points_notes.c`: notes on sequence points and undefined behavior pitfalls.
  
Advanced additions:
- `alignment_and_padding.c`: structure padding, alignment, `offsetof`.
- `endianness.c`: byte order inspection and implications.
- `memory_pool.c`: fixed-size block allocator (free-list) pattern.
- `intrusive_list.c`: intrusive singly linked list without heap allocation.
- `hash_table_linear_probing.c`: open addressing hash table (int→int).
- `dynamic_array.c`: growable vector with doubling strategy.
- `fsm_state_machine.c`: a tiny finite state machine for parsing.
- `minunit.h`, `test_dynamic_array.c`: minimalist unit testing helpers and a sample test.
 - `arena_allocator.c`: bump-pointer arena with mark/reset and alignment.
 - `binary_io_endianness.c`: reading/writing 32-bit values in a portable big-endian format.
 - `adler32.c`: Adler-32 checksum implementation.
 - `cooperative_scheduler.c`: coroutine-like cooperative tasks using program counters.
 - `goto_error_handling.c`: structured cleanup with `goto` for multi-step operations.
  - `mini_printf.c`: tiny printf subset (%s %d %u %x %c and %% ) implemented from scratch.
  - `crc32.c`: CRC-32 checksum with a generated lookup table.
  - `utf8_validate.c`: strict(ish) UTF-8 validation (rejects overlongs, surrogates).
  - `ring_buffer.c`: single-producer/single-consumer circular buffer.
  - `bst_tree.c`: binary search tree insert/search/traverse/free.
  - `json_tokenizer.c`: small tokenizer for JSON-like input.
  - `pack_unpack.c`: manual bit packing/unpacking for protocol bytes.
  - `donut_ascii.c`: classic spinning ASCII donut using incremental rotations.
  - `base64.c`: base64 encoder implementation.
  - `memmove_impl.c`: reimplementing `memmove` with overlap handling.
  - `MAKE_TUTORIAL.md`: a practical guide to GNU Make tailored to this repo.

Optional future (libraries not included in default build):
- OpenGL/GLX demos (Linux): create `opengl_demo/` with its own `Makefile` and package configs.
- SDL2 demos: create `sdl_demo/` with a separate `Makefile`. These will not affect `make run` here.

How to run the optional demos (Linux):
- OpenGL/GLX: `make -C c89_fun/opengl_demo run` (requires `libx11-dev` and `mesa-common-dev`/OpenGL headers)
- SDL2: `make -C c89_fun/sdl_demo run` (requires `libsdl2-dev` and `sdl2-config` on PATH)

Testing note: `test_dynamic_array.c` includes `dynamic_array.c` directly so it can access internal `static` helpers in one translation unit. This keeps the example simple under strict C89 flags.

### How to Study

1) Build and run everything once with `make run`. Skim outputs to see concepts in action.
2) Open a file, read the code, tweak a line (e.g., change values, add prints), rebuild just that file by rerunning `make`, then run only that one binary in `c89_fun/bin/`.
3) Keep notes of surprises or compiler warnings; everything is compiled with strict flags to teach best practices.

### Keep my TODOs

TODO 
- teach me Make
- teach me advanced C
- teach me how to do graphics with C
- Eskil steeberg videos https://www.youtube.com/watch?v=sSpULGNHyoI&t=2494s https://www.youtube.com/watch?v=CxKujAuz2Vw https://www.youtube.com/watch?v=w3_e9vZj7D8 https://www.youtube.com/watch?v=443UNeGrFoM Eskil Steenberg – You should finish your software – BSC 2025

- Jonathan blow
- The big OOPs of programming
- learn advanced language features in general
CC = gcc
CFLAGS = -std=c89 -Wall -Wextra -Werror -pedantic `sdl2-config --cflags`
LIBS = `sdl2-config --libs`

.PHONY: all run clean

all: demo

demo: main.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

run: demo
	./demo

clean:
	rm -f demo



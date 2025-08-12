CC = gcc
CFLAGS = -std=c89 -Wall -Wextra -Werror -pedantic
LIBS = -lX11 -lGL

.PHONY: all run clean

all: demo

demo: main.c
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

run: demo
	./demo

clean:
	rm -f demo



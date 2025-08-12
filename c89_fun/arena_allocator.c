/* C89: bump-pointer arena allocator with mark/reset and simple alignment */
#include <stdio.h>
#include <stdlib.h>

struct Arena {
  unsigned char *base;
  unsigned long capacity;
  unsigned long offset;
};

static void arena_init(struct Arena *a, void *buffer, unsigned long capacity)
{
  a->base = (unsigned char*)buffer;
  a->capacity = capacity;
  a->offset = 0ul;
}


static unsigned long arena_align_up(unsigned long value, unsigned long alignment)
{
  unsigned long mask = alignment - 1ul;
  return (value + mask) & ~mask;
}


static void *arena_alloc(struct Arena *a, unsigned long size, unsigned long alignment)
{
  unsigned long aligned = arena_align_up(a->offset, alignment);
  if (aligned + size > a->capacity) return 0;
  a->offset = aligned + size;
  return (void*)(a->base + aligned);
}


static unsigned long arena_mark(const struct Arena *a)
{
  return a->offset;
}


static void arena_reset_to(struct Arena *a, unsigned long mark)
{
  if (mark <= a->capacity) {
    a->offset = mark;
  }
}


int main(void)
{
  unsigned char storage[256];
  struct Arena arena;
  void *p1;
  void *p2;
  void *p3;
  unsigned long m;

  arena_init(&arena, storage, (unsigned long)sizeof(storage));
  p1 = arena_alloc(&arena, 24ul, 8ul);
  m = arena_mark(&arena);
  p2 = arena_alloc(&arena, 32ul, 16ul);
  p3 = arena_alloc(&arena, 64ul, 32ul);
  printf("p1=%p p2=%p p3=%p offset=%lu\n", p1, p2, p3, arena.offset);

  /* Reset to mark, discarding p2 and p3 allocations */
  arena_reset_to(&arena, m);
  printf("after reset offset=%lu\n", arena.offset);

  /* Allocate again after reset */
  p2 = arena_alloc(&arena, 40ul, 8ul);
  printf("new p2=%p offset=%lu\n", p2, arena.offset);
  return 0;
}




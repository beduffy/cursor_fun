/* C89: arena with scratch scopes for temporary allocations */
#include <stdio.h>
#include <stdlib.h>

struct Arena { unsigned char *base; unsigned long cap; unsigned long off; };

static void arena_init(struct Arena *a, void *buf, unsigned long cap) { a->base=(unsigned char*)buf; a->cap=cap; a->off=0ul; }
static unsigned long arena_mark(const struct Arena *a) { return a->off; }
static void arena_reset(struct Arena *a, unsigned long m) { if (m <= a->cap) a->off = m; }

static void *arena_alloc(struct Arena *a, unsigned long size, unsigned long align)
{
  unsigned long mask = align - 1ul;
  unsigned long aligned = (a->off + mask) & ~mask;
  if (aligned + size > a->cap) return 0;
  a->off = aligned + size;
  return (void*)(a->base + aligned);
}

static void demo_scope(struct Arena *a)
{
  unsigned long m = arena_mark(a);
  char *tmp = (char*)arena_alloc(a, 32ul, 8ul);
  if (tmp) {
    tmp[0]='O'; tmp[1]='K'; tmp[2]='\0';
    printf("scratch: %s\n", tmp);
  }
  arena_reset(a, m);
}

int main(void)
{
  unsigned char buf[128];
  struct Arena a; arena_init(&a, buf, sizeof buf);
  demo_scope(&a);
  if (arena_alloc(&a, 64ul, 8ul) && arena_alloc(&a, 64ul, 8ul) == 0) {
    printf("exhausted as expected\n");
  }
  return 0;
}




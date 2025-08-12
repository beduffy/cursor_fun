/* C89: lock-free style ring buffer (single producer/consumer assumption) */
#include <stdio.h>

#define RB_CAP 8

struct Ring {
  int data[RB_CAP];
  unsigned int head; /* write */
  unsigned int tail; /* read */
};

static int rb_push(struct Ring *r, int v)
{
  unsigned int next = (r->head + 1u) % RB_CAP;
  if (next == r->tail) return 0; /* full */
  r->data[r->head] = v;
  r->head = next;
  return 1;
}

static int rb_pop(struct Ring *r, int *out)
{
  if (r->tail == r->head) return 0; /* empty */
  *out = r->data[r->tail];
  r->tail = (r->tail + 1u) % RB_CAP;
  return 1;
}

int main(void)
{
  struct Ring r; int i, x;
  r.head = 0; r.tail = 0;
  for (i = 0; i < 6; i++) rb_push(&r, i);
  while (rb_pop(&r, &x)) printf("%d ", x);
  printf("\n");
  return 0;
}



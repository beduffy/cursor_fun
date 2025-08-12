/* C89: simple cooperative scheduler (no threads) */
#include <stdio.h>

typedef struct {
  int pc;
  int counter;
} Task;

static int task_print(Task *t)
{
  switch (t->pc) {
    case 0: t->counter = 0; t->pc = 1; return 1;
    case 1: if (t->counter < 3) { printf("task_print %d\n", t->counter++); return 1; }
            t->pc = 2; return 1;
    case 2: return 0; /* done */
  }
  return 0;
}

static int task_count(Task *t)
{
  switch (t->pc) {
    case 0: t->counter = 5; t->pc = 1; return 1;
    case 1: if (t->counter > 0) { printf("task_count %d\n", t->counter--); return 1; }
            t->pc = 2; return 1;
    case 2: return 0;
  }
  return 0;
}

int main(void)
{
  Task a; Task b; int alive;
  a.pc = 0; b.pc = 0; alive = 1;
  while (alive) {
    int ra = task_print(&a);
    int rb = task_count(&b);
    alive = ra || rb;
  }
  return 0;
}



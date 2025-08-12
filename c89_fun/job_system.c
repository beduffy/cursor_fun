/* C89: cooperative job system with priorities and timeslices */
#include <stdio.h>

typedef struct { int pc; int prio; int slice; const char *name; } CTask;

static int task_spin(CTask *t)
{
  switch (t->pc) {
    case 0: t->slice = 2; t->pc = 1; return 1;
    case 1: if (t->slice-- > 0) { printf("%s spin\n", t->name); return 1; } t->pc = 2; return 1;
    case 2: return 0;
  }
  return 0;
}

static int task_countdown(CTask *t)
{
  switch (t->pc) {
    case 0: t->slice = 3; t->pc = 1; return 1;
    case 1: if (t->slice-- > 0) { printf("%s %d\n", t->name, t->slice); return 1; } t->pc = 2; return 1;
    case 2: return 0;
  }
  return 0;
}

int main(void)
{
  CTask a; CTask b; int active = 1; int tick = 0;
  a.pc = 0; a.prio = 1; a.name = "A";
  b.pc = 0; b.prio = 0; b.name = "B";
  while (active) {
    /* simple priority: run lower prio value first */
    int ra = b.prio <= a.prio ? task_countdown(&b) : task_spin(&a);
    int rb = b.prio <= a.prio ? task_spin(&a) : task_countdown(&b);
    active = ra || rb;
    tick++;
    if (tick > 10) break;
  }
  return 0;
}



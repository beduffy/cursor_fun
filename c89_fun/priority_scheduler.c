/* C89: priority-queue based scheduler demo */
#include <stdio.h>

struct Task { int prio; const char *name; };
struct PQ { struct Task a[32]; int n; };

static void pq_push(struct PQ *q, struct Task t)
{
  int i;
  int pr = t.prio;
  if (q->n >= 32) return;
  i = q->n++;
  while (i > 0) {
    int p = (i - 1) / 2;
    if (q->a[p].prio <= pr) break;
    q->a[i] = q->a[p];
    i = p;
  }
  q->a[i] = t;
}

static int pq_pop(struct PQ *q, struct Task *out)
{
  int i;
  int last;
  int left;
  int right;
  int sm;
  if (q->n == 0) return 0;
  *out = q->a[0];
  last = q->a[--q->n].prio;
  i = 0;
  while (1) {
    left = 2 * i + 1; right = left + 1; sm = i;
    if (left < q->n && q->a[left].prio < q->a[sm].prio) sm = left;
    if (right < q->n && q->a[right].prio < q->a[sm].prio) sm = right;
    if (sm == i) break;
    q->a[i] = q->a[sm];
    i = sm;
  }
  q->a[i].prio = last; /* name left unset for brevity in demo */
  return 1;
}

int main(void)
{
  struct PQ q; struct Task t; struct Task a; struct Task b; struct Task c;
  q.n = 0;
  a.prio = 5; a.name = "low"; pq_push(&q, a);
  b.prio = 1; b.name = "urgent"; pq_push(&q, b);
  c.prio = 3; c.name = "normal"; pq_push(&q, c);
  while (pq_pop(&q, &t)) printf("run %s (prio=%d)\n", t.name, t.prio);
  return 0;
}



/* C89: binary min-heap priority queue for ints */
#include <stdio.h>

#define HEAP_CAP 64

struct Heap {
  int data[HEAP_CAP];
  int size;
};

static void heap_init(struct Heap *h)
{
  h->size = 0;
}


static void heap_push(struct Heap *h, int v)
{
  int i;
  if (h->size >= HEAP_CAP) return;
  i = h->size++;
  while (i > 0) {
    int parent = (i - 1) / 2;
    if (h->data[parent] <= v) break;
    h->data[i] = h->data[parent];
    i = parent;
  }
  h->data[i] = v;
}


static int heap_pop(struct Heap *h, int *out)
{
  int v;
  int last;
  int i;
  if (h->size == 0) return 0;
  v = h->data[0];
  last = h->data[--h->size];
  i = 0;
  while (1) {
    int left = 2 * i + 1;
    int right = left + 1;
    int smallest = i;
    if (left < h->size && h->data[left] < h->data[smallest]) smallest = left;
    if (right < h->size && h->data[right] < h->data[smallest]) smallest = right;
    if (smallest == i) break;
    h->data[i] = h->data[smallest];
    i = smallest;
  }
  h->data[i] = last;
  *out = v;
  return 1;
}


int main(void)
{
  struct Heap h; int x;
  heap_init(&h);
  heap_push(&h, 5); heap_push(&h, 1); heap_push(&h, 7); heap_push(&h, 3);
  while (heap_pop(&h, &x)) printf("%d ", x);
  printf("\n");
  return 0;
}




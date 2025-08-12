/* C89: growable dynamic array for ints */
#include <stdio.h>
#include <stdlib.h>

struct IntVec {
  int *data;
  unsigned long size;
  unsigned long capacity;
};

static void vec_init(struct IntVec *v)
{
  v->data = 0;
  v->size = 0;
  v->capacity = 0;
}

static int vec_reserve(struct IntVec *v, unsigned long new_cap)
{
  if (new_cap <= v->capacity) return 1;
  {
    unsigned long cap = v->capacity ? v->capacity : 4;
    int *p;
    while (cap < new_cap) {
      cap = cap * 2;
    }
    p = (int*)realloc(v->data, cap * sizeof(int));
    if (!p) return 0;
    v->data = p;
    v->capacity = cap;
    return 1;
  }
}

static int vec_push(struct IntVec *v, int x)
{
  if (!vec_reserve(v, v->size + 1)) return 0;
  v->data[v->size++] = x;
  return 1;
}

static void vec_free(struct IntVec *v)
{
  free(v->data);
  v->data = 0;
  v->size = 0;
  v->capacity = 0;
}

int main(void)
{
  struct IntVec v;
  unsigned long i;
  vec_init(&v);
  for (i = 0; i < 10; i++) {
    if (!vec_push(&v, (int)(i * 3))) {
      printf("push failed at %lu\n", i);
      vec_free(&v);
      return 0;
    }
  }
  for (i = 0; i < v.size; i++) {
    printf("%d ", v.data[i]);
  }
  printf("\n(capacity=%lu)\n", v.capacity);
  vec_free(&v);
  return 0;
}




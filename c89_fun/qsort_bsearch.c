/* C89: qsort and bsearch */
#include <stdio.h>
#include <stdlib.h>

static int cmp_ints(const void *a, const void *b)
{
  const int *ia = (const int*)a;
  const int *ib = (const int*)b;
  if (*ia < *ib) return -1;
  if (*ia > *ib) return 1;
  return 0;
}


int main(void)
{
  int values[6] = { 42, 7, 13, 99, 0, 7 };
  int i;
  int key = 13;
  int *found;

  qsort(values, (size_t)6, sizeof(int), cmp_ints);
  for (i = 0; i < 6; i++) {
    printf("%d ", values[i]);
  }
  printf("\\n");

  found = (int*)bsearch(&key, values, (size_t)6, sizeof(int), cmp_ints);
  if (found) {
    printf("found %d at index %lu\\n", *found,
           (unsigned long)(found - values));
  } else {
    printf("not found\\n");
  }

  return 0;
}




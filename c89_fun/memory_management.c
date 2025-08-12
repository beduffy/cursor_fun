/* C89: malloc, realloc, free */
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
  int *data;
  int i;
  int count = 5;

  data = (int*)malloc((size_t)count * sizeof(int));
  if (!data) {
    printf("malloc failed\\n");
    return 0;
  }
  for (i = 0; i < count; i++) {
    data[i] = i * i;
  }
  for (i = 0; i < count; i++) {
    printf("%d ", data[i]);
  }
  printf("\\n");

  count = 8;
  data = (int*)realloc(data, (size_t)count * sizeof(int));
  if (!data) {
    printf("realloc failed\\n");
    return 0;
  }
  for (i = 5; i < count; i++) {
    data[i] = i * i;
  }
  for (i = 0; i < count; i++) {
    printf("%d ", data[i]);
  }
  printf("\\n");

  free(data);
  return 0;
}




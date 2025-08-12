/* C89 arrays and pointers, pointer arithmetic */
#include <stdio.h>

int main(void)
{
  int numbers[5];
  int i;
  int *p;
  int sum;

  for (i = 0; i < 5; i++) {
    numbers[i] = (i + 1) * 10; /* 10,20,30,40,50 */
  }

  p = numbers; /* array decays to pointer to first element */
  sum = 0;
  for (i = 0; i < 5; i++) {
    sum += *(p + i);
  }
  printf("sum = %d\\n", sum);

  printf("numbers base address: %p\\n", (void*)numbers);
  printf("p points to: %p, first value=%d\\n", (void*)p, *p);

  return 0;
}



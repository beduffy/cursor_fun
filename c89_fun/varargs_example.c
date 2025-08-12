/* C89: stdarg variable arguments */
#include <stdio.h>
#include <stdarg.h>

static int sum_ints(int count, ...)
{
  va_list ap;
  int i;
  int total = 0;
  va_start(ap, count);
  for (i = 0; i < count; i++) {
    total += va_arg(ap, int);
  }
  va_end(ap);
  return total;
}


int main(void)
{
  int total = sum_ints(5, 1, 2, 3, 4, 5);
  printf("sum=%d\\n", total);
  return 0;
}



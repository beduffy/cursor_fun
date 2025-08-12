/* C89: sequence points (notes). Avoid undefined behavior examples in code. */
#include <stdio.h>

int main(void)
{
  int i = 1;
  int j;

  /* Defined: i is incremented after the assignment */
  j = i++;
  printf("i=%d j=%d\\n", i, j);

  /* Undefined in C89 (do NOT do this): i = i++;  or  a[i] = i++; */
  /* This example is intentionally not compiled to avoid warnings with -Werror. */

  return 0;
}



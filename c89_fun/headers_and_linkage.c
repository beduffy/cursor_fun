/* C89: header usage, prototypes, internal linkage via static */
#include <stdio.h>
#include "mymath.h"

/* Internal linkage: only visible in this translation unit */
static int internal_increment(int x)
{
  return x + 1;
}

/* Provide definitions for the header's prototypes here */
int add_ints(int a, int b)
{
  return a + b;
}


int mul_ints(int a, int b)
{
  return a * b;
}


int main(void)
{
  int x = add_ints(2, 3);
  int y = mul_ints(4, 5);
  int z = internal_increment(9);
  printf("add=%d mul=%d inc=%d\\n", x, y, z);
  return 0;
}



/* C89: function pointers */
#include <stdio.h>

int add(int a, int b) { return a + b; }


int mul(int a, int b) { return a * b; }


int apply(int (*op)(int, int), int x, int y)
{
  return (*op)(x, y);
}


int main(void)
{
  int (*fn)(int, int);
  int r1;
  int r2;

  fn = add;
  r1 = apply(fn, 2, 3);
  fn = mul;
  r2 = apply(fn, 4, 5);
  printf("r1=%d r2=%d\\n", r1, r2);
  return 0;
}




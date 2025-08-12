/* C89: floating point caveats */
#include <stdio.h>
#include <float.h>

int main(void)
{
  double a = 0.1;
  double b = 0.2;
  double c = 0.3;
  double sum = a + b;
  double eps = 10.0 * DBL_EPSILON;
  double diff = sum - c;

  printf("a+b=%.17f, c=%.17f, diff=%.17f\\n", sum, c, diff);
  if (diff < 0) diff = -diff;
  if (diff < eps) {
    printf("close enough within epsilon\\n");
  } else {
    printf("not equal due to rounding\\n");
  }
  return 0;
}




/* C89: integer and floating limits */
#include <stdio.h>
#include <limits.h>
#include <float.h>

int main(void)
{
  printf("CHAR_BIT: %d\\n", CHAR_BIT);
  printf("SCHAR_MIN..SCHAR_MAX: %d..%d\\n", SCHAR_MIN, SCHAR_MAX);
  printf("UCHAR_MAX: %u\\n", (unsigned)UCHAR_MAX);
  printf("SHRT_MIN..SHRT_MAX: %d..%d\\n", SHRT_MIN, SHRT_MAX);
  printf("USHRT_MAX: %u\\n", (unsigned)USHRT_MAX);
  printf("INT_MIN..INT_MAX: %d..%d\\n", INT_MIN, INT_MAX);
  printf("UINT_MAX: %u\\n", (unsigned)UINT_MAX);
  printf("LONG_MIN..LONG_MAX: %ld..%ld\\n", (long)LONG_MIN, (long)LONG_MAX);
  printf("ULONG_MAX: %lu\\n", (unsigned long)ULONG_MAX);

  printf("\\nFloating point: \\\n+FLT_MIN..FLT_MAX: %e..%e, epsilon=%e\\n", FLT_MIN, FLT_MAX, FLT_EPSILON);
  printf("DBL_MIN..DBL_MAX: %e..%e, epsilon=%e\\n", DBL_MIN, DBL_MAX, DBL_EPSILON);

  return 0;
}




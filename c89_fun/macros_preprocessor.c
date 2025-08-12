/* C89 preprocessor: macros, stringizing, do-while trick */
#include <stdio.h>

#define SQUARE(x) ((x) * (x))
#define STR2(x) #x
#define STR(x) STR2(x)

#define LOG_INT(var) \
  do { \
    printf(#var " = %d\n", (var)); \
  } while (0)

int main(void)
{
  int a = 5;
  int s = SQUARE(a + 1); /* expands safely: ((a + 1) * (a + 1)) */
  LOG_INT(a);
  LOG_INT(s);
  printf("This line: %s:%s\\n", __FILE__, STR(__LINE__));
  return 0;
}



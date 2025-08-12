/* C89 basics: hello, types, printf, arithmetic */
#include <stdio.h>

int main(void)
{
  int integerValue = 42;
  double doubleValue = 3.1415926535;
  char letter = 'C';

  printf("Hello, C89!\\n");
  printf("integerValue = %d\\n", integerValue);
  printf("doubleValue  = %.6f\\n", doubleValue);
  printf("letter       = %c\\n", letter);

  integerValue = integerValue + 8; /* 50 */
  printf("integerValue + 8 = %d\\n", integerValue);

  return 0;
}




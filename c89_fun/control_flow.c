/* C89 control flow: if/else, switch, loops */
#include <stdio.h>

int main(void)
{
  int i;
  int sum;
  int value = 3;

  if (value < 0) {
    printf("negative\\n");
  } else if (value == 0) {
    printf("zero\\n");
  } else {
    printf("positive\\n");
  }

  switch (value) {
    case 1: printf("one\\n"); break;
    case 2: printf("two\\n"); break;
    case 3: printf("three\\n"); break;
    default: printf("other\\n"); break;
  }

  sum = 0;
  for (i = 1; i <= 5; i++) {
    sum += i;
  }
  printf("sum 1..5 = %d\\n", sum);

  i = 3;
  while (i > 0) {
    printf("while i=%d\\n", i);
    i--;
  }

  do {
    printf("do-while executes at least once\\n");
  } while (0);

  return 0;
}




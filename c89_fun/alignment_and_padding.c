/* C89: alignment, padding, and offsetof */
#include <stdio.h>
#include <stddef.h> /* offsetof */

struct Mixed {
  char c;
  int i;
  char d;
};

int main(void)
{
  printf("sizeof(char)=%lu sizeof(int)=%lu\\n",
         (unsigned long)sizeof(char), (unsigned long)sizeof(int));
  printf("struct Mixed: sizeof=%lu, offsets: c=%lu i=%lu d=%lu\\n",
         (unsigned long)sizeof(struct Mixed),
         (unsigned long)offsetof(struct Mixed, c),
         (unsigned long)offsetof(struct Mixed, i),
         (unsigned long)offsetof(struct Mixed, d));
  return 0;
}



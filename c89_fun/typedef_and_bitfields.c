/* C89 typedef and bitfields */
#include <stdio.h>

typedef unsigned long ULong;

struct Flags {
  unsigned int a : 1;
  unsigned int b : 2;
  unsigned int c : 3;
};

int main(void)
{
  ULong big = 123456ul;
  struct Flags f;

  f.a = 1; f.b = 2; f.c = 5;

  printf("big=%lu, flags: a=%u b=%u c=%u (sizeof Flags=%lu)\\n",
         big, (unsigned)f.a, (unsigned)f.b, (unsigned)f.c,
         (unsigned long)sizeof(f));

  return 0;
}



/* C89: bitwise ops, masks, shifts */
#include <stdio.h>

int main(void)
{
  unsigned int flags = 0u;
  const unsigned int A = 1u << 0;
  const unsigned int B = 1u << 1;
  const unsigned int C = 1u << 2;

  flags |= A; /* set A */
  flags |= C; /* set C */
  if ((flags & B) == 0u) {
    printf("B not set\\n");
  }

  flags ^= C; /* toggle C */
  printf("flags=0x%X\\n", flags);

  printf("(5 & 3)=%u, (5 | 3)=%u, (5 ^ 3)=%u, (5<<1)=%u, (5>>1)=%u\\n",
         (unsigned)(5 & 3), (unsigned)(5 | 3), (unsigned)(5 ^ 3),
         (unsigned)(5u << 1), (unsigned)(5u >> 1));

  return 0;
}




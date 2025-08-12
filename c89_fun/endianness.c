/* C89: detect endianness via union and byte inspection */
#include <stdio.h>

int main(void)
{
  union { unsigned long u; unsigned char b[sizeof(unsigned long)]; } u;
  unsigned int i;
  u.u = 0x01020304ul;
  printf("unsigned long bytes:");
  for (i = 0; i < sizeof u.b; i++) {
    printf(" %u", (unsigned)u.b[i]);
  }
  printf("\\n");
  if (u.b[0] == 1) {
    printf("big-endian (most significant first)\\n");
  } else if (u.b[0] == 4) {
    printf("little-endian (least significant first)\\n");
  } else {
    printf("mixed/unknown representation\\n");
  }
  return 0;
}




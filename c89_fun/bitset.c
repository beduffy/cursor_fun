/* C89: simple bitset (128 bits) */
#include <stdio.h>

struct Bitset { unsigned long w[2]; };

static void bitset_clear(struct Bitset *b)
{
  b->w[0] = 0ul; b->w[1] = 0ul;
}


static void bitset_set(struct Bitset *b, unsigned int i)
{
  if (i < 64u) b->w[0] |= (1ul << (i & 63u));
  else if (i < 128u) b->w[1] |= (1ul << (i & 63u));
}


static int bitset_test(const struct Bitset *b, unsigned int i)
{
  if (i < 64u) return (b->w[0] >> (i & 63u)) & 1ul;
  else if (i < 128u) return (b->w[1] >> (i & 63u)) & 1ul;
  return 0;
}


int main(void)
{
  struct Bitset b; unsigned int i; int count = 0;
  bitset_clear(&b);
  for (i = 0; i < 128u; i += 3u) bitset_set(&b, i);
  for (i = 0; i < 128u; i++) if (bitset_test(&b, i)) count++;
  printf("bits_set=%d\n", count);
  return 0;
}




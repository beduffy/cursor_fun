/* C89: simple Bloom filter with 1024 bits and 3 hashes */
#include <stdio.h>
#include <string.h>

#define BLOOM_BITS 1024
#define BLOOM_WORDS_32 (BLOOM_BITS/32)
#define WORDS_UL (BLOOM_WORDS_32 / (sizeof(unsigned long)/4)) /* number of unsigned long words */

struct Bloom { unsigned long w[WORDS_UL]; };

static unsigned long h1(const char *s){ unsigned long h=5381ul; while(*s) h=((h<<5)+h)+(unsigned char)*s++; return h; }
static unsigned long h2(const char *s){ unsigned long h=1469598103934665603ul; while(*s){ h^=(unsigned char)*s++; h*=1099511628211ul; } return h; }
static unsigned long h3(const char *s){ unsigned long h=0ul; while(*s){ h = (h*131ul) + (unsigned char)*s++; } return h; }

static void bloom_clear(struct Bloom *b){ unsigned int i; for(i=0;i<WORDS_UL;i++) b->w[i]=0ul; }
static void bloom_set(struct Bloom *b, unsigned int bit){ b->w[(bit>>5)%WORDS_UL] |= (1ul << (bit & 31u)); }
static int  bloom_get(const struct Bloom *b, unsigned int bit){ return (b->w[(bit>>5)%WORDS_UL] >> (bit & 31u)) & 1ul; }

static void bloom_add(struct Bloom *b, const char *s){ bloom_set(b, (unsigned int)(h1(s)%BLOOM_BITS)); bloom_set(b, (unsigned int)(h2(s)%BLOOM_BITS)); bloom_set(b, (unsigned int)(h3(s)%BLOOM_BITS)); }
static int  bloom_maybe(const struct Bloom *b, const char *s){ return bloom_get(b,(unsigned int)(h1(s)%BLOOM_BITS)) && bloom_get(b,(unsigned int)(h2(s)%BLOOM_BITS)) && bloom_get(b,(unsigned int)(h3(s)%BLOOM_BITS)); }

int main(void)
{
  struct Bloom b; bloom_clear(&b);
  bloom_add(&b, "apple"); bloom_add(&b, "banana");
  printf("apple? %d, grape? %d\n", bloom_maybe(&b,"apple"), bloom_maybe(&b,"grape"));
  return 0;
}



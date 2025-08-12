/* C89: Adler-32 checksum (RFC 1950) */
#include <stdio.h>
#include <string.h>

static unsigned long adler32(const unsigned char *data, unsigned long len)
{
  const unsigned long MOD_ADLER = 65521ul;
  unsigned long a = 1ul;
  unsigned long b = 0ul;
  unsigned long i;
  for (i = 0; i < len; i++) {
    a = (a + data[i]) % MOD_ADLER;
    b = (b + a) % MOD_ADLER;
  }
  return (b << 16) | a;
}

int main(void)
{
  const unsigned char *p = (const unsigned char*)"The quick brown fox jumps over the lazy dog";
  unsigned long sum = adler32(p, (unsigned long)strlen((const char*)p));
  printf("adler32=0x%08lX\n", sum);
  return 0;
}



/* C89: CRC-32 (IEEE 802.3) with a generated lookup table */
#include <stdio.h>

static unsigned long crc_table[256];

static void crc32_init(void)
{
  unsigned long poly = 0xEDB88320ul;
  unsigned int i;
  for (i = 0; i < 256; i++) {
    unsigned long c = (unsigned long)i;
    unsigned int j;
    for (j = 0; j < 8; j++) {
      if (c & 1ul) c = poly ^ (c >> 1);
      else c >>= 1;
    }
    crc_table[i] = c;
  }
}


static unsigned long crc32(const unsigned char *data, unsigned long len)
{
  unsigned long c = 0xFFFFFFFFul;
  unsigned long i;
  for (i = 0; i < len; i++) {
    c = crc_table[(c ^ data[i]) & 0xFFul] ^ (c >> 8);
  }
  return c ^ 0xFFFFFFFFul;
}


int main(void)
{
  const unsigned char *p = (const unsigned char*)"123456789";
  unsigned long sum;
  crc32_init();
  sum = crc32(p, 9ul);
  printf("crc32=0x%08lX\n", sum); /* should be 0xCBF43926 */
  return 0;
}




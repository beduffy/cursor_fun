/* C89: endianness-safe binary I/O for 32-bit values (manual byte order) */
#include <stdio.h>
#include <stdlib.h>

static void u32_to_be(unsigned long x, unsigned char out[4])
{
  out[0] = (unsigned char)((x >> 24) & 0xFFul);
  out[1] = (unsigned char)((x >> 16) & 0xFFul);
  out[2] = (unsigned char)((x >>  8) & 0xFFul);
  out[3] = (unsigned char)( x        & 0xFFul);
}

static unsigned long be_to_u32(const unsigned char in[4])
{
  return ((unsigned long)in[0] << 24) |
         ((unsigned long)in[1] << 16) |
         ((unsigned long)in[2] <<  8) |
         ((unsigned long)in[3]);
}

int main(void)
{
  const char *path = "/tmp/c89_be.bin";
  FILE *fp;
  unsigned long values_out[3];
  unsigned long values_in[3];
  unsigned char buf[4];
  int i;

  values_out[0] = 0x11223344ul;
  values_out[1] = 0xAABBCCDDul;
  values_out[2] = 0x01020304ul;

  fp = fopen(path, "wb");
  if (!fp) { printf("open write failed: %s\n", path); return 0; }
  for (i = 0; i < 3; i++) {
    u32_to_be(values_out[i], buf);
    fwrite(buf, 1u, 4u, fp);
  }
  fclose(fp);

  fp = fopen(path, "rb");
  if (!fp) { printf("open read failed: %s\n", path); return 0; }
  for (i = 0; i < 3; i++) {
    if (fread(buf, 1u, 4u, fp) == 4u) {
      values_in[i] = be_to_u32(buf);
    }
  }
  fclose(fp);

  for (i = 0; i < 3; i++) {
    printf("0x%08lX\n", values_in[i]);
  }
  return 0;
}



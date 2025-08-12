/* C89: byte Run-Length Encoding (RLE) encode/decode demo */
#include <stdio.h>
#include <string.h>

static unsigned long rle_encode(const unsigned char *in, unsigned long n, unsigned char *out)
{
  unsigned long i = 0ul;
  unsigned long j = 0ul;
  while (i < n) {
    unsigned char b = in[i];
    unsigned long run = 1ul;
    while (i + run < n && in[i + run] == b && run < 255ul) run++;
    out[j++] = (unsigned char)run;
    out[j++] = b;
    i += run;
  }
  return j;
}


static unsigned long rle_decode(const unsigned char *in, unsigned long n, unsigned char *out)
{
  unsigned long i = 0ul;
  unsigned long j = 0ul;
  while (i + 1ul < n) {
    unsigned char run = in[i++];
    unsigned char b = in[i++];
    while (run--) out[j++] = b;
  }
  return j;
}


int main(void)
{
  const unsigned char *msg = (const unsigned char*)"aaabbbcccccccccczz";
  unsigned char enc[128];
  unsigned char dec[128];
  unsigned long en = rle_encode(msg, (unsigned long)strlen((const char*)msg), enc);
  unsigned long dn = rle_decode(enc, en, dec);
  dec[dn] = 0;
  printf("enc_bytes=%lu dec=%s\n", en, dec);
  return 0;
}



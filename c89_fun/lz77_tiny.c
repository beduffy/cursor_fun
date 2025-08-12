/* C89: tiny naive LZ77-like compressor/decompressor (educational) */
#include <stdio.h>
#include <string.h>

#define WIN 32
#define MAXLEN 15

/* Output format: sequence of tokens. Each token starts with a flag byte.
   If flag==0: next byte is literal. If flag==1: next two bytes are (offset,len). */

static int compress_lz77(const unsigned char *in, int n, unsigned char *out)
{
  int i = 0; int j = 0;
  while (i < n) {
    int bestLen = 0, bestOff = 0;
    int wstart = i - WIN; if (wstart < 0) wstart = 0;
    {
      int pos;
      for (pos = wstart; pos < i; pos++) {
        int len = 0;
        while (len < MAXLEN && i+len < n && in[pos+len] == in[i+len]) len++;
        if (len > bestLen) { bestLen = len; bestOff = i - pos; }
      }
    }
    if (bestLen >= 3) {
      out[j++] = 1; out[j++] = (unsigned char)bestOff; out[j++] = (unsigned char)bestLen; i += bestLen;
    } else {
      out[j++] = 0; out[j++] = in[i++];
    }
  }
  return j;
}

static int decompress_lz77(const unsigned char *in, int n, unsigned char *out)
{
  int i = 0; int j = 0;
  while (i < n) {
    if (in[i++] == 0) { out[j++] = in[i++]; }
    else {
      int off = in[i++]; int len = in[i++]; int k;
      for (k = 0; k < len; k++) out[j] = out[j - off], j++;
    }
  }
  return j;
}

int main(void)
{
  const unsigned char *msg = (const unsigned char*)"ABABABABABABABABA XYZ XYZ XYZ";
  unsigned char enc[256]; unsigned char dec[256];
  int en = compress_lz77(msg, (int)strlen((const char*)msg), enc);
  int dn = decompress_lz77(enc, en, dec); dec[dn] = 0;
  printf("enc=%d dec=%s\n", en, dec);
  return 0;
}




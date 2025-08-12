/* C89: very small LZ4-like (not compatible) illustrative encoder/decoder */
#include <stdio.h>
#include <string.h>

/* Token: [lit_len(4 bits)][match_len(4 bits)] then literals, then offset(2B), then extra len */

static int min(int a, int b){ return a<b?a:b; }

static int encode(const unsigned char *in, int n, unsigned char *out)
{
  int i=0, j=0;
  while (i < n) {
    int lit_len = 0;
    int match_len = 0;
    int best_off = 0;
    int wi = i - 64;
    int k;
    if (wi < 0) wi = 0;
    /* find match */
    for (k = wi; k < i; k++) {
      int off = i - k; int m=0;
      while (i+m < n && in[k+m] == in[i+m] && m < 15+255) m++;
      if (m > match_len) { match_len = m; best_off = off; }
    }
    if (match_len >= 4) {
      int token = (0 << 4) | (min(match_len-4, 15));
      out[j++] = (unsigned char)token;
      out[j++] = (unsigned char)(best_off & 255);
      out[j++] = (unsigned char)((best_off >> 8) & 255);
      if (match_len - 4 >= 15) { int rem = match_len - 4 - 15; while (rem > 0) { int step=min(rem,255); out[j++]=(unsigned char)step; rem -= step; } }
      i += match_len;
    } else {
      /* accumulate literals until a match is found or cap */
      int start = i; lit_len = 1; i++;
      while (i < n) {
        int wi2 = i - 64; int found=0; int k2; if (wi2 < 0) wi2 = 0;
        for (k2 = wi2; k2 < i && !found; k2++) {
          int m2=0; while (i+m2 < n && in[k2+m2] == in[i+m2] && m2 < 4) m2++;
          if (m2 >= 4) found = 1;
        }
        if (found || lit_len >= 15) { break; }
        lit_len++;
        i++;
      }
      out[j++] = (unsigned char)((min(lit_len,15) << 4) | 0);
      if (lit_len >= 15) { int rem = lit_len - 15; while (rem > 0) { int step=min(rem,255); out[j++]=(unsigned char)step; rem -= step; } }
      memcpy(out+j, in+start, (size_t)lit_len); j += lit_len;
    }
  }
  return j;
}

static int decode(const unsigned char *in, int n, unsigned char *out)
{
  int i=0, j=0;
  while (i < n) {
    int token = in[i++];
    int lit = (token >> 4) & 15;
    int mlen = (token & 15) + 4;
    if (lit == 15) { int s; do { s=in[i++]; lit += s; } while (s == 255 && i < n); }
    memcpy(out+j, in+i, (size_t)lit); j += lit; i += lit;
    if (i >= n) break;
    {
      int lo = in[i++];
      int hi = in[i++];
      int off = lo | (hi<<8);
      int k2;
      for (k2=0; k2<mlen; k2++) { out[j] = out[j - off]; j++; }
    }
    if ((token & 15) == 15) { int s2; do { s2=in[i++]; mlen += s2; } while (s2 == 255 && i < n); }
  }
  return j;
}

int main(void)
{
  const unsigned char *msg = (const unsigned char*)"LZ4 LZ4 LZ4 LZ4!!! 123123123";
  unsigned char enc[512]; unsigned char dec[512];
  int en = encode(msg, (int)strlen((const char*)msg), enc);
  int dn = decode(enc, en, dec); dec[dn] = 0;
  printf("lz4lite enc=%d dec=%s\n", en, dec);
  return 0;
}



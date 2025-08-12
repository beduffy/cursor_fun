/* C89: base64 decode */
#include <stdio.h>
#include <string.h>

static int b64_index(char c)
{
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

static int decode(const char *in, unsigned char *out, int outcap)
{
  int i = 0; int j = 0; int len = (int)strlen(in);
  while (i < len) {
    int v[4]; int k;
    for (k = 0; k < 4 && i < len; k++, i++) {
      int idx;
      if (in[i] == '=') { v[k] = -2; continue; }
      idx = b64_index(in[i]);
      if (idx < 0) { k--; continue; }
      v[k] = idx;
    }
    if (k < 4) break;
    if (j + 3 <= outcap) {
      if (v[0] == -2 || v[1] == -2) break;
      out[j++] = (unsigned char)((v[0] << 2) | (v[1] >> 4));
      if (v[2] != -2) {
        out[j++] = (unsigned char)(((v[1] & 15) << 4) | (v[2] >> 2));
        if (v[3] != -2) {
          out[j++] = (unsigned char)(((v[2] & 3) << 6) | v[3]);
        }
      }
    }
  }
  return j;
}

int main(void)
{
  const char *s = "Qzg5IGJhc2U2NCE=";
  unsigned char out[64];
  int n = decode(s, out, 64);
  out[n] = '\0';
  printf("%s\n", out);
  return 0;
}




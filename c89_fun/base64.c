/* C89: base64 encode/decode */
#include <stdio.h>
#include <string.h>

static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void encode(const unsigned char *in, unsigned long len)
{
  unsigned long i;
  for (i = 0; i < len; i += 3) {
    unsigned long v = 0;
    int rem = (int)(len - i);
    v |= (unsigned long)in[i] << 16;
    if (rem > 1) v |= (unsigned long)in[i+1] << 8;
    if (rem > 2) v |= (unsigned long)in[i+2];
    putchar(b64[(v >> 18) & 63]);
    putchar(b64[(v >> 12) & 63]);
    putchar(rem > 1 ? b64[(v >> 6) & 63] : '=');
    putchar(rem > 2 ? b64[v & 63] : '=');
  }
  putchar('\n');
}

int main(void)
{
  const unsigned char *msg = (const unsigned char*)"C89 base64!";
  encode(msg, (unsigned long)strlen((const char*)msg));
  return 0;
}



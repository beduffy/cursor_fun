/* C89: minimal UTF-8 validator (no normalization; rejects overlongs) */
#include <stdio.h>

static int utf8_valid(const unsigned char *s)
{
  while (*s) {
    if (*s < 0x80u) { s++; continue; }
    else if ((*s & 0xE0u) == 0xC0u) { /* 2-byte */
      unsigned char a = s[0], b = s[1];
      if ((b & 0xC0u) != 0x80u) return 0;
      if (a < 0xC2u) return 0; /* no overlong */
      s += 2;
    } else if ((*s & 0xF0u) == 0xE0u) { /* 3-byte */
      unsigned char a = s[0], b = s[1], c = s[2];
      if ((b & 0xC0u) != 0x80u || (c & 0xC0u) != 0x80u) return 0;
      if (a == 0xE0u && b < 0xA0u) return 0; /* overlong */
      if (a == 0xEDu && b >= 0xA0u) return 0; /* surrogates */
      s += 3;
    } else if ((*s & 0xF8u) == 0xF0u) { /* 4-byte */
      unsigned char a = s[0], b = s[1], c = s[2], d = s[3];
      if ((b & 0xC0u) != 0x80u || (c & 0xC0u) != 0x80u || (d & 0xC0u) != 0x80u) return 0;
      if (a > 0xF4u) return 0; /* > U+10FFFF */
      if (a == 0xF0u && b < 0x90u) return 0; /* overlong */
      if (a == 0xF4u && b > 0x8Fu) return 0; /* > U+10FFFF */
      s += 4;
    } else {
      return 0;
    }
  }
  return 1;
}

int main(void)
{
  const unsigned char ok[] = "hello \xF0\x9F\x98\x80"; /* grinning face */
  const unsigned char bad[] = "\xC0\xAF"; /* overlong slash */
  printf("ok? %d\n", utf8_valid(ok));
  printf("bad? %d\n", utf8_valid(bad));
  return 0;
}



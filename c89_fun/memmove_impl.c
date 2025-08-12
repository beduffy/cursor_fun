/* C89: implement memmove semantics (overlap-safe copy) */
#include <stdio.h>
#include <string.h>

static void *my_memmove(void *dst, const void *src, unsigned long n)
{
  unsigned char *d = (unsigned char*)dst;
  const unsigned char *s = (const unsigned char*)src;
  if (d == s || n == 0ul) return dst;
  if (d < s) {
    unsigned long i;
    for (i = 0; i < n; i++) d[i] = s[i];
  } else {
    unsigned long i = n;
    while (i--) d[i] = s[i];
  }
  return dst;
}

int main(void)
{
  char buf[32];
  strcpy(buf, "abcdefghijklmnop");
  my_memmove(buf+2, buf, 8);
  printf("%s\n", buf);
  return 0;
}




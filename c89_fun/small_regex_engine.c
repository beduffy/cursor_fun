/* C89: tiny regex-like engine supporting ^ $ . * and literal chars */
#include <stdio.h>

static int match_here(const char *re, const char *s);

static int match_star(int c, const char *re, const char *s)
{
  do {
    if (match_here(re, s)) return 1;
  } while (*s != '\0' && (*s++ == c || c == '.'));
  return 0;
}

static int match_here(const char *re, const char *s)
{
  if (re[0] == '\0') return 1;
  if (re[1] == '*') return match_star(re[0], re+2, s);
  if (re[0] == '$' && re[1] == '\0') return *s == '\0';
  if (*s != '\0' && (re[0] == '.' || re[0] == *s)) return match_here(re+1, s+1);
  return 0;
}

static int match(const char *re, const char *s)
{
  if (re[0] == '^') return match_here(re+1, s);
  do {
    if (match_here(re, s)) return 1;
  } while (*s++ != '\0');
  return 0;
}

int main(void)
{
  printf("%d %d %d %d\n", match("^ab.*d$", "abxyzqd"), match("h.llo", "hello"), match("a*b", "aaab"), match("^c$", "c"));
  return 0;
}



/* C89: simple glob wildcard match (* and ?) */
#include <stdio.h>

static int match_here(const char *pat, const char *str)
{
  while (*pat && *pat != '*') {
    if (*pat != '?' && *pat != *str) return 0;
    if (*str == '\0') return 0;
    pat++; str++;
  }
  if (*pat == '\0') return *str == '\0';
  /* pat now at '*' */
  while (*pat == '*') pat++;
  if (*pat == '\0') return 1; /* trailing * matches all */
  while (*str) {
    if (match_here(pat, str)) return 1;
    str++;
  }
  return 0;
}

int main(void)
{
  printf("%d %d %d %d\n",
         match_here("a*bc", "axxxbc"),
         match_here("a?c", "abc"),
         match_here("*", "hello"),
         match_here("h?llo", "hillo"));
  return 0;
}




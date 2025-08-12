/* C89: Knuth-Morris-Pratt substring search */
#include <stdio.h>
#include <string.h>

static void compute_lps(const char *pat, int m, int *lps)
{
  int len = 0;
  int i;
  lps[0] = 0;
  i = 1;
  while (i < m) {
    if (pat[i] == pat[len]) {
      len++;
      lps[i] = len;
      i++;
    } else {
      if (len != 0) {
        len = lps[len - 1];
      } else {
        lps[i] = 0;
        i++;
      }
    }
  }
}


int main(void)
{
  const char *text = "ababcabcabababd";
  const char *pat = "ababd";
  int n = (int)strlen(text);
  int m = (int)strlen(pat);
  int lps[64];
  int i;
  int j;
  compute_lps(pat, m, lps);
  i = 0; j = 0;
  while (i < n) {
    if (pat[j] == text[i]) { i++; j++; }
    if (j == m) {
      printf("match at %d\n", i - j);
      j = lps[j - 1];
    } else if (i < n && pat[j] != text[i]) {
      if (j != 0) j = lps[j - 1]; else i++;
    }
  }
  return 0;
}




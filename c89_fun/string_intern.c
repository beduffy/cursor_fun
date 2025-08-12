/* C89: simple string interning with a fixed-size table */
#include <stdio.h>
#include <string.h>

#define MAX_STR 64
#define TABLE 128

static char pool[TABLE][MAX_STR];
static int used[TABLE];

static unsigned int hash_str(const char *s)
{
  unsigned int h = 2166136261u; /* FNV-1a */
  while (*s) { h ^= (unsigned char)*s++; h *= 16777619u; }
  return h;
}

static const char *intern(const char *s)
{
  unsigned int idx = hash_str(s) % TABLE;
  unsigned int i;
  for (i=0;i<TABLE;i++) {
    unsigned int j = (idx + i) % TABLE;
    if (used[j] && strcmp(pool[j], s) == 0) return pool[j];
    if (!used[j]) { strncpy(pool[j], s, MAX_STR-1); pool[j][MAX_STR-1]='\0'; used[j]=1; return pool[j]; }
  }
  return s; /* fallback */
}

int main(void)
{
  const char *a = intern("hello");
  const char *b = intern("world");
  const char *c = intern("hello");
  printf("a==c? %d, a='%s', b='%s'\n", (a==c), a, b);
  return 0;
}




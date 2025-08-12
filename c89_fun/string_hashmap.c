/* C89: minimal string->int hashmap with linear probing (uses own strdup) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAP 257

struct Entry { char *k; int v; int used; };
static struct Entry tab[CAP];

static unsigned long hstr(const char *s)
{
  unsigned long h = 5381ul; while (*s){ h = ((h<<5) + h) + (unsigned char)*s++; } return h;
}

static char *dup_cstr(const char *s)
{
  size_t n = strlen(s); char *p = (char*)malloc(n+1u); if (!p) return 0; memcpy(p,s,n); p[n]='\0'; return p;
}

static int put(const char *k, int v)
{
  unsigned long idx = hstr(k) % CAP; unsigned long i;
  for (i=0;i<CAP;i++){
    unsigned long j = (idx + i) % CAP;
    if (!tab[j].used || (tab[j].k && strcmp(tab[j].k,k)==0)){
      if (!tab[j].used) { tab[j].k = dup_cstr(k); if (!tab[j].k) return 0; tab[j].used = 1; }
      tab[j].v = v; return 1;
    }
  }
  return 0;
}

static int get(const char *k, int *out)
{
  unsigned long idx = hstr(k) % CAP; unsigned long i;
  for (i=0;i<CAP;i++){
    unsigned long j = (idx + i) % CAP; if (!tab[j].used) return 0; if (tab[j].k && strcmp(tab[j].k,k)==0){ *out=tab[j].v; return 1; }
  }
  return 0;
}

int main(void)
{
  int v;
  put("alpha", 1); put("beta", 2); put("gamma", 3);
  if (get("beta", &v)) printf("beta=%d\n", v);
  if (!get("delta", &v)) printf("delta not found\n");
  return 0;
}



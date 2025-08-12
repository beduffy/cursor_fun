/* C89: disjoint-set union (union-find) with path compression and union by rank */
#include <stdio.h>

#define N 10

static int parent[N];
static int rankv[N];

static void uf_init(void)
{
  int i; for (i = 0; i < N; i++) { parent[i] = i; rankv[i] = 0; }
}

static int uf_find(int x)
{
  int r = x; while (r != parent[r]) r = parent[r];
  /* Path compression */
  while (x != r) { int p = parent[x]; parent[x] = r; x = p; }
  return r;
}

static void uf_union(int a, int b)
{
  int ra = uf_find(a), rb = uf_find(b);
  if (ra == rb) return;
  if (rankv[ra] < rankv[rb]) parent[ra] = rb;
  else if (rankv[ra] > rankv[rb]) parent[rb] = ra;
  else { parent[rb] = ra; rankv[ra]++; }
}

int main(void)
{
  uf_init();
  uf_union(1, 2); uf_union(2, 3); uf_union(5, 6);
  printf("1~3? %d\n", uf_find(1) == uf_find(3));
  printf("1~5? %d\n", uf_find(1) == uf_find(5));
  uf_union(3, 5);
  printf("1~6? %d\n", uf_find(1) == uf_find(6));
  return 0;
}



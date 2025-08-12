/* C89: BFS and DFS on adjacency list (static) */
#include <stdio.h>

#define N 6

static int adj[N][N] = {
  {0,1,1,0,0,0},
  {1,0,1,1,0,0},
  {1,1,0,0,1,0},
  {0,1,0,0,1,1},
  {0,0,1,1,0,1},
  {0,0,0,1,1,0}
};

static void bfs(int start)
{
  int q[N]; int head=0, tail=0; int vis[N]; int i;
  for (i=0;i<N;i++) vis[i]=0;
  vis[start]=1; q[tail++]=start;
  while (head<tail) {
    int v=q[head++];
    printf("%d ", v);
    for (i=0;i<N;i++) if (adj[v][i] && !vis[i]) { vis[i]=1; q[tail++]=i; }
  }
  printf("\n");
}

static void dfs_rec(int v, int vis[])
{
  int i; vis[v]=1; printf("%d ", v);
  for (i=0;i<N;i++) if (adj[v][i] && !vis[i]) dfs_rec(i, vis);
}

static void dfs(int start)
{
  int vis[N]; int i; for (i=0;i<N;i++) vis[i]=0; dfs_rec(start, vis); printf("\n");
}

int main(void)
{
  bfs(0); dfs(0);
  return 0;
}




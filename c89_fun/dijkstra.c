/* C89: Dijkstra shortest paths on a small graph (adjacency matrix) */
#include <stdio.h>

#define N 5
#define INF 1000000000

static void dijkstra(const int graph[N][N], int src, int dist[N])
{
  int visited[N];
  int i, j;
  for (i = 0; i < N; i++) { dist[i] = INF; visited[i] = 0; }
  dist[src] = 0;
  for (i = 0; i < N; i++) {
    int u = -1; int best = INF;
    for (j = 0; j < N; j++) {
      if (!visited[j] && dist[j] < best) { best = dist[j]; u = j; }
    }
    if (u < 0) break;
    visited[u] = 1;
    for (j = 0; j < N; j++) {
      int w = graph[u][j];
      if (w >= 0 && dist[u] + w < dist[j]) {
        dist[j] = dist[u] + w;
      }
    }
  }
}


int main(void)
{
  int g[N][N] = {
    {  0,  2,  5, -1, -1 },
    {  2,  0,  1,  2, -1 },
    {  5,  1,  0,  3,  1 },
    { -1,  2,  3,  0,  2 },
    { -1, -1,  1,  2,  0 }
  };
  int dist[N];
  int i;
  dijkstra(g, 0, dist);
  for (i = 0; i < N; i++) printf("%d ", dist[i]);
  printf("\n");
  return 0;
}



/* C89: A* pathfinding on a small grid */
#include <stdio.h>

#define W 10
#define H 10
#define NODES (W*H)

static int grid[H][W] = {
  {0,0,0,0,0,0,0,0,0,0},
  {0,1,1,1,0,0,0,1,1,0},
  {0,0,0,1,0,1,0,0,1,0},
  {0,1,0,1,0,1,0,1,1,0},
  {0,1,0,0,0,1,0,0,0,0},
  {0,1,1,1,0,1,0,1,1,0},
  {0,0,0,1,0,0,0,0,1,0},
  {0,1,0,1,1,1,1,0,1,0},
  {0,1,0,0,0,0,0,0,0,0},
  {0,0,0,1,1,1,0,1,1,0}
};

static int abs_i(int x){ return x<0?-x:x; }
static int idx(int x,int y){ return y*W+x; }

int main(void)
{
  int open[NODES]; int g[NODES]; int f[NODES]; int came[NODES]; int closed[NODES];
  int i;
  int sx=0, sy=0, tx=9, ty=9;
  for(i=0;i<NODES;i++){ open[i]=0; g[i]=9999; f[i]=9999; came[i]=-1; closed[i]=0; }
  {
    int s = idx(sx,sy); g[s]=0; f[s]=abs_i(sx-tx)+abs_i(sy-ty); open[s]=1;
  }
  while (1){
    int best=-1; int bestf=9999;
    for(i=0;i<NODES;i++) if(open[i] && f[i]<bestf){ bestf=f[i]; best=i; }
    if (best==-1) break; /* no path */
    open[best]=0; closed[best]=1;
    if (best==idx(tx,ty)) break; /* reached */
    {
      int bx = best%W, by=best/W; int d;
      static const int dx[4]={1,-1,0,0}; static const int dy[4]={0,0,1,-1};
      for(d=0; d<4; d++){
        int nx=bx+dx[d], ny=by+dy[d]; int ni;
        if (nx<0 || ny<0 || nx>=W || ny>=H) { continue; }
        if (grid[ny][nx]) { continue; }
        ni = idx(nx,ny);
        if(closed[ni]) continue;
        if(g[best]+1 < g[ni]){ g[ni]=g[best]+1; f[ni]=g[ni]+abs_i(nx-tx)+abs_i(ny-ty); came[ni]=best; open[ni]=1; }
      }
    }
  }
  {
    int cur=idx(tx,ty); int path[NODES]; int plen=0;
    while(cur!=-1){ path[plen++]=cur; cur=came[cur]; }
    if(plen>0 && path[plen-1]==idx(sx,sy)){
      int p; for(p=plen-1;p>=0;p--){ int x=path[p]%W, y=path[p]/W; grid[y][x]=2; }
      for(i=0;i<H;i++){ int x; for(x=0;x<W;x++){ int v=grid[i][x]; char c = v==1?'#':(v==2?'*':'.'); putchar(c);} putchar('\n'); }
    } else {
      printf("no path\n");
    }
  }
  return 0;
}



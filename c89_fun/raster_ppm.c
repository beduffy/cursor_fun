/* C89: simple 2D raster to PPM (lines via Bresenham, filled triangle) */
#include <stdio.h>

#define W 160
#define H 120

static unsigned char img[W*H*3];

static void clear(unsigned char r, unsigned char g, unsigned char b)
{
  int i; for (i=0; i<W*H; i++){ img[3*i+0]=r; img[3*i+1]=g; img[3*i+2]=b; }
}

static void put_px(int x,int y,unsigned char r,unsigned char g,unsigned char b)
{
  if (x<0 || y<0 || x>=W || y>=H) {
    return;
  }
  img[3*(y*W+x)+0]=r; img[3*(y*W+x)+1]=g; img[3*(y*W+x)+2]=b;
}

static void line(int x0,int y0,int x1,int y1,unsigned char r,unsigned char g,unsigned char b)
{
  int dx = x1 - x0; int dy = y1 - y0;
  int sx = dx>=0?1:-1; int sy = dy>=0?1:-1; dx = dx>=0?dx:-dx; dy = dy>=0?dy:-dy;
  put_px(x0,y0,r,g,b);
  if (dx > dy) {
    int err = dx/2; while (x0 != x1) { x0 += sx; err -= dy; if (err < 0){ y0 += sy; err += dx; } put_px(x0,y0,r,g,b);}    
  } else {
    int err = dy/2; while (y0 != y1) { y0 += sy; err -= dx; if (err < 0){ x0 += sx; err += dy; } put_px(x0,y0,r,g,b);}    
  }
}

static void swap_int(int *a,int *b){ int t=*a; *a=*b; *b=t; }

static void fill_flat_bottom(int x0,int y0,int x1,int y1,int x2,int y2,unsigned char r,unsigned char g,unsigned char b)
{
  double invslope1 = (double)(x1 - x0) / (double)(y1 - y0);
  double invslope2 = (double)(x2 - x0) / (double)(y2 - y0);
  double curx1 = (double)x0; double curx2 = (double)x0; int y;
  for (y = y0; y <= y1; y++) {
    int x; int a = (int)curx1; int c = (int)curx2; if (a>c) { int t=a;a=c;c=t; }
    for (x=a; x<=c; x++) put_px(x,y,r,g,b);
    curx1 += invslope1; curx2 += invslope2;
  }
}

static void fill_flat_top(int x0,int y0,int x1,int y1,int x2,int y2,unsigned char r,unsigned char g,unsigned char b)
{
  double invslope1 = (double)(x2 - x0) / (double)(y2 - y0);
  double invslope2 = (double)(x2 - x1) / (double)(y2 - y1);
  double curx1 = (double)x2; double curx2 = (double)x2; int y;
  for (y = y2; y >= y0; y--) {
    int x; int a = (int)curx1; int c = (int)curx2; if (a>c) { int t=a;a=c;c=t; }
    for (x=a; x<=c; x++) put_px(x,y,r,g,b);
    curx1 -= invslope1; curx2 -= invslope2;
  }
}

static void fill_triangle(int x0,int y0,int x1,int y1,int x2,int y2,unsigned char r,unsigned char g,unsigned char b)
{
  /* sort by y */
  if (y0>y1){ swap_int(&y0,&y1); swap_int(&x0,&x1);} if (y1>y2){ swap_int(&y1,&y2); swap_int(&x1,&x2);} if (y0>y1){ swap_int(&y0,&y1); swap_int(&x0,&x1);}    
  if (y1 == y2) { fill_flat_bottom(x0,y0,x1,y1,x2,y2,r,g,b); }
  else if (y0 == y1) { fill_flat_top(x0,y0,x1,y1,x2,y2,r,g,b); }
  else {
    int x3 = x0 + (int)((double)(y1 - y0) * (double)(x2 - x0) / (double)(y2 - y0)); int y3 = y1;
    fill_flat_bottom(x0,y0,x1,y1,x3,y3,r,g,b);
    fill_flat_top(x1,y1,x3,y3,x2,y2,r,g,b);
  }
}

int main(void)
{
  const char *path = "/tmp/c89_raster.ppm";
  FILE *fp; int x;
  clear(20,20,30);
  /* grid */
  for (x=0; x<W; x+=10) line(x,0,x,H-1,40,40,60);
  for (x=0; x<H; x+=10) line(0,x,W-1,x,40,40,60);
  /* primitives */
  line(5,5, 140, 90, 255,200,60);
  fill_triangle(30,20, 120,25, 90,90, 80,180,80);
  /* write PPM */
  fp = fopen(path, "wb"); if(!fp){ printf("failed to write\n"); return 0; }
  fprintf(fp, "P6\n%d %d\n255\n", W,H);
  fwrite(img,1u,(unsigned)(W*H*3),fp); fclose(fp);
  printf("wrote %s\n", path);
  return 0;
}



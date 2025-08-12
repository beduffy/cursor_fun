/* C89: draw a simple architecture diagram to a PNG using a tiny 5x7 font */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* --- minimal PNG helpers (stored block + adler32 + crc32) --- */
static unsigned long crc_table[256];
static void crc32_init(void){ unsigned long p=0xEDB88320ul; unsigned int i,j; for(i=0;i<256;i++){ unsigned long c=i; for(j=0;j<8;j++) c=(c&1ul)?(p^(c>>1)):(c>>1); crc_table[i]=c; } }
static unsigned long crc32_calc(const unsigned char *d, unsigned long n){ unsigned long c=0xFFFFFFFFul,i; for(i=0;i<n;i++) c=crc_table[(c^d[i])&255]^ (c>>8); return c^0xFFFFFFFFul; }
static unsigned long adler32(const unsigned char *d, unsigned long n){ const unsigned long MOD=65521ul; unsigned long a=1ul,b=0ul,i; for(i=0;i<n;i++){ a=(a+d[i])%MOD; b=(b+a)%MOD; } return (b<<16)|a; }
static void u32be(FILE *fp, unsigned long v){ unsigned char b[4]; b[0]=v>>24; b[1]=v>>16; b[2]=v>>8; b[3]=v; fwrite(b,1u,4u,fp); }
static void chunk(FILE *fp, const char t[4], const unsigned char *data, unsigned long len){ unsigned char th[4]; unsigned long crc; unsigned char *buf; unsigned long i; u32be(fp,len); th[0]=t[0];th[1]=t[1];th[2]=t[2];th[3]=t[3]; fwrite(th,1u,4u,fp); if(len) fwrite(data,1u,len,fp); buf=(unsigned char*)malloc(len+4u); if(!buf){ return; } for(i=0;i<4;i++) buf[i]=th[i]; if(len) memcpy(buf+4,data,len); crc=crc32_calc(buf,len+4u); free(buf); u32be(fp,crc); }

/* --- tiny 5x7 font for selected uppercase letters and space --- */
static unsigned char font5x7(char ch, int row)
{
  /* Each row is 5 bits wide packed into LSBs; define for required letters */
  static const unsigned char A[7]={0x1E,0x11,0x1F,0x11,0x11,0x11,0x00};
  static const unsigned char B[7]={0x1E,0x11,0x1E,0x11,0x11,0x1E,0x00};
  static const unsigned char Cc[7]={0x0E,0x11,0x10,0x10,0x11,0x0E,0x00};
  static const unsigned char D[7]={0x1C,0x12,0x11,0x11,0x12,0x1C,0x00};
  static const unsigned char E[7]={0x1F,0x10,0x1E,0x10,0x10,0x1F,0x00};
  static const unsigned char F[7]={0x1F,0x10,0x1E,0x10,0x10,0x10,0x00};
  static const unsigned char G[7]={0x0F,0x10,0x13,0x11,0x11,0x0F,0x00};
  static const unsigned char H[7]={0x11,0x11,0x1F,0x11,0x11,0x11,0x00};
  static const unsigned char I[7]={0x1F,0x04,0x04,0x04,0x04,0x1F,0x00};
  static const unsigned char J[7]={0x1F,0x02,0x02,0x02,0x12,0x0C,0x00};
  static const unsigned char L[7]={0x10,0x10,0x10,0x10,0x10,0x1F,0x00};
  static const unsigned char M[7]={0x11,0x1B,0x15,0x11,0x11,0x11,0x00};
  static const unsigned char N[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x00};
  static const unsigned char O[7]={0x0E,0x11,0x11,0x11,0x11,0x0E,0x00};
  static const unsigned char Rr[7]={0x1E,0x11,0x1E,0x14,0x12,0x11,0x00};
  static const unsigned char S[7]={0x0F,0x10,0x0E,0x01,0x11,0x0E,0x00};
  static const unsigned char V[7]={0x11,0x11,0x11,0x11,0x0A,0x04,0x00};
  static const unsigned char Y[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x00};
  static const unsigned char Z[7]={0x1F,0x01,0x02,0x04,0x08,0x1F,0x00};
  static const unsigned char space[7]={0,0,0,0,0,0,0};
  switch(ch){
    case 'A': return A[row]; case 'B': return B[row]; case 'C': return Cc[row];
    case 'D': return D[row]; case 'E': return E[row]; case 'F': return F[row];
    case 'G': return G[row]; case 'H': return H[row]; case 'I': return I[row];
    case 'J': return J[row]; case 'L': return L[row]; case 'M': return M[row];
    case 'N': return N[row]; case 'O': return O[row]; case 'R': return Rr[row];
    case 'S': return S[row]; case 'V': return V[row]; case 'Y': return Y[row];
    case 'Z': return Z[row]; case ' ': return space[row]; default: return space[row];
  }
}

/* --- canvas and drawing --- */
#define W 640
#define H 400
static unsigned char img[W*H*3];
static void px(int x,int y,unsigned char r,unsigned char g,unsigned char b){ if(x<0||y<0||x>=W||y>=H) return; img[3*(y*W+x)+0]=r; img[3*(y*W+x)+1]=g; img[3*(y*W+x)+2]=b; }
static void clear(unsigned char r,unsigned char g,unsigned char b){ int i; for(i=0;i<W*H;i++){ img[3*i+0]=r; img[3*i+1]=g; img[3*i+2]=b; } }
static void rect(int x0,int y0,int x1,int y1,unsigned char r,unsigned char g,unsigned char b){ int x,y; for(x=x0;x<=x1;x++){ px(x,y0,r,g,b); px(x,y1,r,g,b);} for(y=y0;y<=y1;y++){ px(x0,y,r,g,b); px(x1,y,r,g,b);} }
static void fill(int x0,int y0,int x1,int y1,unsigned char r,unsigned char g,unsigned char b){ int x,y; for(y=y0;y<=y1;y++) for(x=x0;x<=x1;x++) px(x,y,r,g,b); }
static void text5x7(int x,int y,const char *s,unsigned char r,unsigned char g,unsigned char b){ int cx=x; int i; while(*s){ char ch=*s++; for(i=0;i<7;i++){ unsigned char row=font5x7(ch,i); int bit; for(bit=0;bit<5;bit++){ if(row & (1<<(4-bit))) px(cx+bit,y+i,r,g,b); } } cx+=6; } }
static void hline(int x0,int x1,int y,unsigned char r,unsigned char g,unsigned char b){ int x; for(x=x0;x<=x1;x++) px(x,y,r,g,b); }
static void vline(int x,int y0,int y1,unsigned char r,unsigned char g,unsigned char b){ int y; for(y=y0;y<=y1;y++) px(x,y,r,g,b); }

static void write_png(const char *path)
{
  unsigned long raw_len = (unsigned long)H * (1u + (unsigned long)W * 3u);
  unsigned char *raw = (unsigned char*)malloc(raw_len);
  unsigned char *scan = raw; unsigned long y; unsigned char *zbuf; unsigned long zlen; FILE *fp;
  crc32_init();
  for(y=0;y<(unsigned long)H;y++){
    unsigned long off = y*(unsigned long)W*3u;
    *scan++ = 0; /* filter */
    memcpy(scan, img+off, (unsigned long)W*3u);
    scan += (unsigned long)W*3u;
  }
  zlen = 2u + 5u + raw_len + 4u;
  zbuf = (unsigned char*)malloc(zlen);
  if(!zbuf){ free(raw); return; }
  zbuf[0]=0x78; zbuf[1]=0x01;
  {
    unsigned long p=2u; unsigned long len=raw_len; unsigned long nlen=(~len)&0xFFFFul;
    zbuf[p++]=0x01; zbuf[p++]=len&255; zbuf[p++]=(len>>8)&255; zbuf[p++]=nlen&255; zbuf[p++]=(nlen>>8)&255;
    memcpy(zbuf+p, raw, raw_len); p+=raw_len; {
      unsigned long ad=adler32(raw, raw_len); zbuf[p++]=(ad>>24)&255; zbuf[p++]=(ad>>16)&255; zbuf[p++]=(ad>>8)&255; zbuf[p++]=ad&255;
    }
  }
  fp = fopen(path,"wb"); if(!fp){ free(zbuf); free(raw); return; }
  { unsigned char sig[8]={137,80,78,71,13,10,26,10}; fwrite(sig,1u,8u,fp);} {
    unsigned char ihdr[13]; ihdr[0]=(W>>24)&255; ihdr[1]=(W>>16)&255; ihdr[2]=(W>>8)&255; ihdr[3]=W&255; ihdr[4]=(H>>24)&255; ihdr[5]=(H>>16)&255; ihdr[6]=(H>>8)&255; ihdr[7]=H&255; ihdr[8]=8; ihdr[9]=2; ihdr[10]=0; ihdr[11]=0; ihdr[12]=0; chunk(fp,"IHDR",ihdr,13u); }
  chunk(fp,"IDAT", zbuf, zlen);
  chunk(fp,"IEND", 0, 0);
  fclose(fp); free(zbuf); free(raw);
}

int main(void)
{
  const char *out = "/tmp/c89_arch.png";
  clear(245,246,250);
  /* Rows of boxes */
  fill(30,20, 200,70, 230,236,250); rect(30,20,200,70, 60,80,120); text5x7(40,40,"BUILD", 20,30,80);
  fill(230,20, 420,70, 230,236,250); rect(230,20,420,70, 60,80,120); text5x7(240,40,"CORE", 20,30,80);
  fill(450,20, 610,70, 230,236,250); rect(450,20,610,70, 60,80,120); text5x7(460,40,"MEMORY", 20,30,80);

  fill(30,100, 200,150, 230,236,250); rect(30,100,200,150, 60,80,120); text5x7(40,120,"IO", 20,30,80);
  fill(230,100, 420,150, 230,236,250); rect(230,100,420,150, 60,80,120); text5x7(240,120,"DS", 20,30,80);
  fill(450,100, 610,150, 230,236,250); rect(450,100,610,150, 60,80,120); text5x7(460,120,"ALGO", 20,30,80);

  fill(30,180, 200,230, 230,236,250); rect(30,180,200,230, 60,80,120); text5x7(40,200,"JSON", 20,30,80);
  fill(230,180, 420,230, 230,236,250); rect(230,180,420,230, 60,80,120); text5x7(240,200,"SCHED", 20,30,80);
  fill(450,180, 610,230, 230,236,250); rect(450,180,610,230, 60,80,120); text5x7(460,200,"VM", 20,30,80);

  fill(230,260, 420,310, 230,236,250); rect(230,260,420,310, 60,80,120); text5x7(240,280,"FUN", 20,30,80);

  /* connectors */
  hline(115,115,75, 120,140,180); vline(115,75,95, 120,140,180);
  hline(315,315,75, 120,140,180); vline(315,75,95, 120,140,180);
  hline(535,535,75, 120,140,180); vline(535,75,95, 120,140,180);

  vline(115,155,175, 120,140,180);
  vline(315,155,175, 120,140,180);
  vline(535,155,175, 120,140,180);

  vline(315,235,255, 120,140,180);

  write_png(out);
  printf("wrote %s\n", out);
  return 0;
}



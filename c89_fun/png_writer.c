/* C89: minimal PNG (RGB8) writer using zlib stored block + Adler-32 + CRC32 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* CRC32 (IEEE) table */
static unsigned long crc_table[256];

static void crc32_init(void)
{
  unsigned long poly = 0xEDB88320ul;
  unsigned int i, j; unsigned long c;
  for (i = 0; i < 256; i++) { c = (unsigned long)i; for (j=0;j<8;j++) c = (c & 1ul) ? (poly ^ (c >> 1)) : (c >> 1); crc_table[i] = c; }
}

static unsigned long crc32_calc(const unsigned char *data, unsigned long len)
{
  unsigned long c = 0xFFFFFFFFul; unsigned long i;
  for (i = 0; i < len; i++) c = crc_table[(c ^ data[i]) & 0xFFul] ^ (c >> 8);
  return c ^ 0xFFFFFFFFul;
}

static unsigned long adler32(const unsigned char *data, unsigned long len)
{
  const unsigned long MOD = 65521ul; unsigned long a = 1ul, b = 0ul; unsigned long i;
  for (i = 0; i < len; i++) { a = (a + data[i]) % MOD; b = (b + a) % MOD; }
  return (b << 16) | a;
}

static void write_u32_be(FILE *fp, unsigned long v)
{ unsigned char b[4]; b[0]=(unsigned char)(v>>24); b[1]=(unsigned char)(v>>16); b[2]=(unsigned char)(v>>8); b[3]=(unsigned char)v; fwrite(b,1u,4u,fp); }

static void write_chunk(FILE *fp, const char type[4], const unsigned char *data, unsigned long len)
{
  unsigned char hdr[4]; unsigned long crc;
  write_u32_be(fp, len);
  hdr[0]=type[0]; hdr[1]=type[1]; hdr[2]=type[2]; hdr[3]=type[3];
  fwrite(hdr,1u,4u,fp);
  if (len) fwrite(data,1u,len,fp);
  /* CRC over type+data */
  {
    unsigned char *buf; unsigned long i;
    buf = (unsigned char*)malloc(len+4u);
    if (!buf) return; /* best-effort */
    for (i=0;i<4;i++) buf[i]=hdr[i];
    if (len) memcpy(buf+4, data, len);
    crc = crc32_calc(buf, len+4u);
    free(buf);
  }
  write_u32_be(fp, crc);
}

int main(void)
{
  const int W = 64; const int H = 64;
  unsigned char *scan; unsigned long raw_len; unsigned long zlen;
  unsigned char *raw; unsigned char *zbuf; unsigned long y, x;
  const char *path = "/tmp/c89_demo.png";
  FILE *fp;
  crc32_init();

  /* Prepare raw scanlines with filter byte 0 */
  raw_len = (unsigned long)H * (1u + (unsigned long)W * 3u);
  raw = (unsigned char*)malloc(raw_len);
  if (!raw) return 0;
  scan = raw;
  for (y=0; y<(unsigned long)H; y++) {
    *scan++ = 0; /* filter type 0 */
    for (x=0; x<(unsigned long)W; x++) {
      unsigned char r = (unsigned char)(x * 255u / (W-1));
      unsigned char g = (unsigned char)(y * 255u / (H-1));
      unsigned char b = (unsigned char)128u;
      *scan++ = r; *scan++ = g; *scan++ = b;
    }
  }

  /* Build zlib stream with single stored block */
  /* CMF/FLG with no compression: 0x78 0x01 */
  zlen = 2u /* zlib hdr */ + 5u /* block header */ + raw_len + 4u /* adler */;
  zbuf = (unsigned char*)malloc(zlen);
  if (!zbuf) { free(raw); return 0; }
  zbuf[0]=0x78; zbuf[1]=0x01;
  {
    unsigned long p = 2u; unsigned long len = raw_len; unsigned long nlen = (~len) & 0xFFFFul;
    /* final block + stored */
    zbuf[p++] = 0x01; /* BFINAL=1,BTYPE=00 */
    zbuf[p++] = (unsigned char)(len & 0xFFul);
    zbuf[p++] = (unsigned char)((len >> 8) & 0xFFul);
    zbuf[p++] = (unsigned char)(nlen & 0xFFul);
    zbuf[p++] = (unsigned char)((nlen >> 8) & 0xFFul);
    memcpy(zbuf+p, raw, raw_len); p += raw_len;
    /* adler32 of raw data */
    {
      unsigned long ad = adler32(raw, raw_len);
      zbuf[p++] = (unsigned char)((ad >> 24) & 0xFFul);
      zbuf[p++] = (unsigned char)((ad >> 16) & 0xFFul);
      zbuf[p++] = (unsigned char)((ad >> 8) & 0xFFul);
      zbuf[p++] = (unsigned char)(ad & 0xFFul);
    }
  }

  fp = fopen(path, "wb"); if (!fp) { free(zbuf); free(raw); return 0; }
  /* PNG signature */
  {
    unsigned char sig[8] = {137,80,78,71,13,10,26,10}; fwrite(sig,1u,8u,fp);
  }
  /* IHDR */
  {
    unsigned char ihdr[13];
    ihdr[0]=(unsigned char)((W>>24)&255); ihdr[1]=(unsigned char)((W>>16)&255); ihdr[2]=(unsigned char)((W>>8)&255); ihdr[3]=(unsigned char)(W&255);
    ihdr[4]=(unsigned char)((H>>24)&255); ihdr[5]=(unsigned char)((H>>16)&255); ihdr[6]=(unsigned char)((H>>8)&255); ihdr[7]=(unsigned char)(H&255);
    ihdr[8]=8; /* bit depth */ ihdr[9]=2; /* color type RGB */ ihdr[10]=0; ihdr[11]=0; ihdr[12]=0;
    write_chunk(fp, "IHDR", ihdr, 13u);
  }
  /* IDAT */ write_chunk(fp, "IDAT", zbuf, zlen);
  /* IEND */ write_chunk(fp, "IEND", 0, 0);
  fclose(fp);
  free(zbuf); free(raw);
  printf("wrote %s\n", path);
  return 0;
}



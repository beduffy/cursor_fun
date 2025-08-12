/* C89: write a simple 16-bit PCM WAV with a few tones */
#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void write_u32_le(FILE *fp, unsigned long v){ unsigned char b[4]; b[0]=v&255; b[1]=(v>>8)&255; b[2]=(v>>16)&255; b[3]=(v>>24)&255; fwrite(b,1u,4u,fp);} 
static void write_u16_le(FILE *fp, unsigned int v){ unsigned char b[2]; b[0]=v&255; b[1]=(v>>8)&255; fwrite(b,1u,2u,fp);} 

int main(void)
{
  const unsigned long sr = 44100;
  const unsigned long samples = sr * 2ul; /* 2 seconds */
  const char *path = "/tmp/c89_tone.wav";
  FILE *fp = fopen(path, "wb"); unsigned long i; if(!fp) return 0;
  /* RIFF header */
  fwrite("RIFF",1u,4u,fp); write_u32_le(fp, 36u + samples*2u); fwrite("WAVE",1u,4u,fp);
  /* fmt chunk */
  fwrite("fmt ",1u,4u,fp); write_u32_le(fp, 16u); write_u16_le(fp, 1u); write_u16_le(fp, 1u); write_u32_le(fp, sr); write_u32_le(fp, sr*2u); write_u16_le(fp, 2u); write_u16_le(fp, 16u);
  /* data */
  fwrite("data",1u,4u,fp); write_u32_le(fp, samples*2u);
  for (i=0;i<samples;i++){
    double t = (double)i / (double)sr;
    double freq = (i < sr) ? 440.0 : 660.0; /* A4 then E5 */
    double s = sin(2.0 * M_PI * freq * t);
    int v = (int)(s * 30000.0);
    write_u16_le(fp, (unsigned int)(v & 0xFFFF));
  }
  fclose(fp);
  printf("wrote %s\n", path);
  return 0;
}



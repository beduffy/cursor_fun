/* C89: pack/unpack bitfields manually into bytes (protocol-style) */
#include <stdio.h>

/* Layout: [3 bits A][5 bits B] [8 bits C] = 2 bytes total */

static void pack(unsigned int A, unsigned int B, unsigned int C, unsigned char out[2])
{
  unsigned int v = ((A & 0x7u) << 13) | ((B & 0x1Fu) << 8) | (C & 0xFFu);
  out[0] = (unsigned char)((v >> 8) & 0xFFu);
  out[1] = (unsigned char)(v & 0xFFu);
}

static void unpack(const unsigned char in[2], unsigned int *A, unsigned int *B, unsigned int *C)
{
  unsigned int v = ((unsigned int)in[0] << 8) | (unsigned int)in[1];
  *A = (v >> 13) & 0x7u;
  *B = (v >> 8) & 0x1Fu;
  *C = v & 0xFFu;
}

int main(void)
{
  unsigned char buf[2];
  unsigned int A1=5, B1=17, C1=200, A2, B2, C2;
  pack(A1, B1, C1, buf);
  unpack(buf, &A2, &B2, &C2);
  printf("%u %u %u -> [%u %u] -> %u %u %u\n", A1, B1, C1, (unsigned)buf[0], (unsigned)buf[1], A2, B2, C2);
  return 0;
}



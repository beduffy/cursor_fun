/* C89: simple Q16.16 fixed-point arithmetic */
#include <stdio.h>

typedef long fx; /* 16.16 signed fixed-point, stored in 32-bit long */

#define FX_ONE ((fx)1 << 16)

static int fx_to_int(fx x) { return (int)(x >> 16); }

static fx fx_from_float_approx(double d)
{
  /* crude conversion without libm */
  long whole = (long)d;
  double frac = d - (double)whole;
  long f = (long)(frac * 65536.0);
  return (((fx)whole) << 16) + (fx)f;
}

static fx fx_mul(fx a, fx b)
{
  /* Use double for safe intermediate in C89 without long long */
  double r = ((double)a * (double)b) / 65536.0;
  if (r > 2147483647.0) r = 2147483647.0;
  if (r < -2147483648.0) r = -2147483648.0;
  return (fx)r;
}

static fx fx_div(fx a, fx b)
{
  double r = ((double)a * 65536.0) / (double)b;
  if (r > 2147483647.0) r = 2147483647.0;
  if (r < -2147483648.0) r = -2147483648.0;
  return (fx)r;
}

int main(void)
{
  fx a = fx_from_float_approx(3.5);
  fx b = fx_from_float_approx(2.25);
  fx c = fx_mul(a, b);
  fx d = fx_div(a, b);
  printf("3.5*2.25 ~= %d.%04d\n", fx_to_int(c), (int)((c & 0xFFFF) * 10000 / 65536));
  printf("3.5/2.25 ~= %d.%04d\n", fx_to_int(d), (int)((d & 0xFFFF) * 10000 / 65536));
  return 0;
}



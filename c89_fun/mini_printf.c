/* C89: minimal printf implementation for %s %d %u %x %c and %% */
#include <stdio.h>
#include <stdarg.h>

static void out_char(int ch) { putchar(ch); }

static void out_cstring(const char *s)
{
  while (*s) { out_char((unsigned char)*s); s++; }
}

static void out_uint_hex(unsigned long v, int base, int is_signed)
{
  char buf[32];
  const char *digits = "0123456789abcdef";
  int i = 0;
  unsigned long x;
  if (is_signed && (long)v < 0) {
    out_char('-');
    x = (unsigned long)(-(long)v);
  } else {
    x = v;
  }
  if (x == 0) { out_char('0'); return; }
  while (x && i < (int)(sizeof buf)) {
    unsigned long d = x % (unsigned long)base;
    buf[i++] = digits[d];
    x /= (unsigned long)base;
  }
  while (i--) out_char(buf[i]);
}

static void mini_vprintf(const char *fmt, va_list ap)
{
  while (*fmt) {
    if (*fmt != '%') { out_char((unsigned char)*fmt++); continue; }
    fmt++;
    switch (*fmt) {
      case '%': out_char('%'); break;
      case 's': { const char *s = va_arg(ap, const char*); if (!s) s = "(null)"; out_cstring(s); } break;
      case 'c': { int ch = va_arg(ap, int); out_char(ch); } break;
      case 'd': { long v = va_arg(ap, int); out_uint_hex((unsigned long)v, 10, 1); } break;
      case 'u': { unsigned long v = va_arg(ap, unsigned int); out_uint_hex(v, 10, 0); } break;
      case 'x': { unsigned long v = va_arg(ap, unsigned int); out_uint_hex(v, 16, 0); } break;
      default: out_char('%'); out_char((unsigned char)*fmt); break;
    }
    fmt++;
  }
}

static void mini_printf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  mini_vprintf(fmt, ap);
  va_end(ap);
}

int main(void)
{
  mini_printf("hello %s %d %u 0x%x %c %%\n", "world", -42, 42u, 0xBEEF, 'Z');
  return 0;
}



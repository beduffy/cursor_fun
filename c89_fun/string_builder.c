/* C89: simple growable string builder */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct StrBuf {
  char *data;
  unsigned long size;
  unsigned long cap;
};

static void sb_init(struct StrBuf *sb)
{
  sb->data = 0;
  sb->size = 0;
  sb->cap = 0;
}


static int sb_reserve(struct StrBuf *sb, unsigned long need)
{
  unsigned long new_cap;
  char *p;
  if (need <= sb->cap) return 1;
  new_cap = sb->cap == 0 ? 64ul : sb->cap;
  while (new_cap < need) {
    new_cap = new_cap * 2ul;
  }
  p = (char*)realloc(sb->data, new_cap);
  if (!p) return 0;
  sb->data = p;
  sb->cap = new_cap;
  return 1;
}


static int sb_append_cstr(struct StrBuf *sb, const char *s)
{
  unsigned long n;
  unsigned long need;
  if (!s) return 1;
  n = (unsigned long)strlen(s);
  need = sb->size + n + 1ul;
  if (!sb_reserve(sb, need)) return 0;
  memcpy(sb->data + sb->size, s, n);
  sb->size += n;
  sb->data[sb->size] = '\0';
  return 1;
}


static int sb_append_char(struct StrBuf *sb, char c)
{
  unsigned long need = sb->size + 2ul;
  if (!sb_reserve(sb, need)) return 0;
  sb->data[sb->size++] = c;
  sb->data[sb->size] = '\0';
  return 1;
}


static void sb_free(struct StrBuf *sb)
{
  free(sb->data);
  sb->data = 0;
  sb->size = 0;
  sb->cap = 0;
}


int main(void)
{
  struct StrBuf sb;
  sb_init(&sb);
  sb_append_cstr(&sb, "Hello");
  sb_append_char(&sb, ',');
  sb_append_char(&sb, ' ');
  sb_append_cstr(&sb, "C89!");
  printf("%s\n", sb.data ? sb.data : "");
  sb_free(&sb);
  return 0;
}



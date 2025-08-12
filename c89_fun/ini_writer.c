/* C89: tiny INI writer */
#include <stdio.h>

static int write_kv(FILE *fp, const char *k, const char *v)
{
  return fprintf(fp, "%s=%s\n", k, v) > 0;
}

int main(void)
{
  const char *path = "/tmp/c89_out.ini";
  FILE *fp = fopen(path, "w");
  if (!fp) { printf("open failed\n"); return 0; }
  fprintf(fp, "[core]\n");
  write_kv(fp, "name", "demo");
  write_kv(fp, "count", "3");
  fprintf(fp, "\n[paths]\n");
  write_kv(fp, "root", "/tmp");
  fclose(fp);
  printf("wrote %s\n", path);
  return 0;
}




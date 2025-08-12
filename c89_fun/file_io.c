/* C89: stdio file I/O */
#include <stdio.h>

int main(void)
{
  const char *path = "/tmp/c89_demo.txt";
  FILE *fp;
  char buffer[64];

  fp = fopen(path, "w");
  if (!fp) {
    printf("Failed to open for write: %s\\n", path);
    return 0;
  }
  fprintf(fp, "line one\\nline two\\n");
  fclose(fp);

  fp = fopen(path, "r");
  if (!fp) {
    printf("Failed to open for read: %s\\n", path);
    return 0;
  }
  while (fgets(buffer, sizeof(buffer), fp)) {
    printf("read: %s", buffer);
  }
  fclose(fp);

  return 0;
}



/* C89: structured error handling with goto for cleanup */
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
  FILE *fp = 0;
  char *buf = 0;
  int rc = 1;

  fp = fopen("/tmp/c89_goto_demo.txt", "w");
  if (!fp) {
    printf("failed to open file\n");
    goto cleanup;
  }

  buf = (char*)malloc(128u);
  if (!buf) {
    printf("malloc failed\n");
    goto cleanup;
  }

  if (fputs("hello\n", fp) < 0) {
    printf("write failed\n");
    goto cleanup;
  }

  rc = 0; /* success */

cleanup:
  if (fp) fclose(fp);
  free(buf);
  if (rc == 0) printf("ok\n");
  return rc;
}



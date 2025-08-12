/* C89 strings: char arrays, strlen, strcpy (careful!), fgets */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(void)
{
  char hello[32];
  size_t len;

  /* Safe copy because destination is large enough */
  strcpy(hello, "Hello, world");
  len = strlen(hello);
  printf("%s (length %lu)\\n", hello, (unsigned long)len);

  /* Safer input: fgets instead of gets. To avoid blocking during `make run`,
     this block runs only if C89_INTERACTIVE=1 is set in the environment. */
  if (getenv("C89_INTERACTIVE") != 0 && strcmp(getenv("C89_INTERACTIVE"), "1") == 0) {
    char input[16];
    printf("Type a short word (<= 15 chars): ");
    if (fgets(input, sizeof(input), stdin)) {
      /* Strip trailing newline if present */
      size_t n = strlen(input);
      if (n > 0 && input[n - 1] == '\n') {
        input[n - 1] = '\0';
      }
      printf("You typed: '%s'\\n", input);
    } else {
      printf("No input read.\\n");
    }
  }

  return 0;
}



/* C89: time, rand, errno */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>

int main(void)
{
  time_t now = time((time_t*)0);
  int i;
  int denom;
  int res;

  printf("time()=%ld\\n", (long)now);
  srand((unsigned)now);
  for (i = 0; i < 3; i++) {
    printf("rand=%d\\n", rand());
  }

  errno = 0;
  denom = 0;
  res = 0;
  if (denom == 0) {
    errno = EDOM; /* pretend domain error */
  } else {
    res = 10 / denom;
  }
  if (errno != 0) {
    printf("errno set to %d\\n", errno);
  } else {
    printf("res=%d\\n", res);
  }

  return 0;
}




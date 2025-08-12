/* C89: measure CPU time using clock() */
#include <stdio.h>
#include <time.h>

static volatile unsigned long sink = 0ul;

static void busy_work(void)
{
  unsigned long i; for (i = 0; i < 10000000ul; i++) sink += i & 1ul;
}


int main(void)
{
  clock_t start = clock();
  busy_work();
  {
    double secs = (double)(clock() - start) / (double)CLOCKS_PER_SEC;
    printf("busy_work took %.3f s (sink=%lu)\n", secs, sink);
  }
  return 0;
}




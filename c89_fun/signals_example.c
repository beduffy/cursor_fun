/* C89: basic signals */
#include <stdio.h>
#include <signal.h>

static void handler(int signum)
{
  printf("caught signal %d\\n", signum);
}


int main(void)
{
  /* Use SIGINT which is present in strictly conforming C environments */
  if (signal(SIGINT, handler) == SIG_ERR) {
    printf("signal setup failed\\n");
    return 0;
  }
  raise(SIGINT);
  return 0;
}



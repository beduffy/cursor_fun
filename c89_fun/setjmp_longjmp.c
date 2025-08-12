/* C89: setjmp/longjmp control transfer */
#include <stdio.h>
#include <setjmp.h>

static jmp_buf env;

static void jump_now(void)
{
  longjmp(env, 42);
}


int main(void)
{
  int code = setjmp(env);
  if (code == 0) {
    printf("first pass, will longjmp...\\n");
    jump_now();
  } else {
    printf("returned via longjmp with code=%d\\n", code);
  }
  return 0;
}



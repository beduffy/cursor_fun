/* C89: const and volatile basics */
#include <stdio.h>

static int length_of_const_string(const char *s)
{
  int n = 0;
  while (s[n] != '\0') {
    n++;
  }
  return n;
}


int main(void)
{
  const char *message = "constant";
  volatile int tick = 0; /* pretend hardware-updated */
  int i;

  printf("len('%s')=%d\\n", message, length_of_const_string(message));

  for (i = 0; i < 3; i++) {
    /* Read from volatile each time to prevent unwanted optimization */
    int snapshot = tick;
    printf("tick snapshot: %d\\n", snapshot);
    tick = snapshot + 1;
  }

  return 0;
}




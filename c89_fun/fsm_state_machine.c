/* C89: simple finite state machine (FSM) */
#include <stdio.h>

enum State { S_IDLE, S_NUMBER, S_DONE };

int main(void)
{
  const char *input = "42a";
  enum State st = S_IDLE;
  int value = 0;
  const char *p = input;

  while (st != S_DONE) {
    char ch = *p;
    switch (st) {
      case S_IDLE:
        if (ch >= '0' && ch <= '9') {
          value = ch - '0';
          st = S_NUMBER;
          p++;
        } else {
          st = S_DONE;
        }
        break;
      case S_NUMBER:
        if (ch >= '0' && ch <= '9') {
          value = value * 10 + (ch - '0');
          p++;
        } else {
          st = S_DONE;
        }
        break;
      default:
        st = S_DONE;
        break;
    }
  }
  printf("parsed=%d from '%s'\n", value, input);
  return 0;
}



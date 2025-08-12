/* C89: tiny stack-based bytecode VM */
#include <stdio.h>

enum { OP_PUSH = 1, OP_ADD = 2, OP_MUL = 3, OP_PRINT = 4, OP_HALT = 0 };

int main(void)
{
  unsigned char code[] = {
    OP_PUSH, 3,
    OP_PUSH, 4,
    OP_ADD,
    OP_PUSH, 2,
    OP_MUL,
    OP_PRINT,
    OP_HALT
  };
  int ip = 0; int stack[16]; int sp = 0; int running = 1;
  while (running) {
    unsigned char op = code[ip++];
    switch (op) {
      case OP_PUSH: stack[sp++] = (int)code[ip++]; break;
      case OP_ADD: { int b=stack[--sp]; int a=stack[--sp]; stack[sp++]=a+b; } break;
      case OP_MUL: { int b=stack[--sp]; int a=stack[--sp]; stack[sp++]=a*b; } break;
      case OP_PRINT: printf("%d\n", stack[sp-1]); break;
      case OP_HALT: running = 0; break;
      default: running = 0; break;
    }
  }
  return 0;
}



/* C89: write JSON from the tiny AST (reuses parser TU) */
#include <stdio.h>

/* Pull in AST types and parser */
#define main json_ast_parser_demo_main
#include "json_ast_parser.c"
#undef main

static void write_json(const JNode *n)
{
  int i;
  switch (n->kind) {
    case J_STR: {
      const char *s = n->str; putchar('"'); while (*s) { if (*s=='"'||*s=='\\') putchar('\\'); putchar(*s++); } putchar('"');
    } break;
    case J_NUM: printf("%g", n->num); break;
    case J_ARR:
      putchar('[');
      for (i=0;i<n->len;i++) { if (i) putchar(','); write_json(n->elems[i]); }
      putchar(']');
      break;
    case J_OBJ:
      putchar('{');
      for (i=0;i<n->len;i++) {
        if (i) putchar(',');
        {
          const char *k = n->pairs[i].key; putchar('"'); while (*k) { if (*k=='"'||*k=='\\') putchar('\\'); putchar(*k++);} putchar('"');
        }
        putchar(':');
        write_json(n->pairs[i].val);
      }
      putchar('}');
      break;
    default: printf("null"); break;
  }
}

int main(void)
{
  /* Parse, then re-emit JSON */
  p = "{\"x\": [1,2,3], \"y\": \"hi\"}";
  write_json(parse_value());
  putchar('\n');
  return 0;
}



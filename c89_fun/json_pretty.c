/* C89: pretty-print JSON using the tiny AST (reuses parser TU) */
#include <stdio.h>

/* Include AST and parser */
#define main json_ast_parser_demo_main
#include "json_ast_parser.c"
#undef main

static void indent(int n) { int i; for (i=0;i<n;i++) putchar(' '); }

static void pretty_write(const JNode *n, int depth)
{
  int i;
  switch (n->kind) {
    case J_STR: {
      const char *s = n->str; putchar('"'); while (*s) { if (*s=='"'||*s=='\\') putchar('\\'); putchar(*s++); } putchar('"');
    } break;
    case J_NUM: printf("%g", n->num); break;
    case J_ARR:
      puts("[");
      for (i=0;i<n->len;i++) {
        indent(depth+2);
        pretty_write(n->elems[i], depth+2);
        if (i+1<n->len) putchar(',');
        putchar('\n');
      }
      indent(depth); putchar(']');
      break;
    case J_OBJ:
      puts("{");
      for (i=0;i<n->len;i++) {
        const char *k;
        indent(depth+2);
        k = n->pairs[i].key; putchar('"'); while (*k) { if (*k=='"'||*k=='\\') putchar('\\'); putchar(*k++);} putchar('"');
        printf(": ");
        pretty_write(n->pairs[i].val, depth+2);
        if (i+1<n->len) putchar(',');
        putchar('\n');
      }
      indent(depth); putchar('}');
      break;
    default: printf("null"); break;
  }
}

int main(void)
{
  p = "{\"menu\":{\"id\":\"file\",\"items\":[\"new\",\"open\",{\"sub\":[1,2]}]}}";
  pretty_write(parse_value(), 0);
  putchar('\n');
  return 0;
}



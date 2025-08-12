/* C89: simple JSON key query using the tiny AST (includes parser TU) */
#include <stdio.h>
#include <string.h>

/* Include the AST parser implementation into this TU */
#define main json_ast_parser_demo_main
#include "json_ast_parser.c"
#undef main

static const JNode *find_key_in_object(const JNode *obj, const char *key)
{
  int i;
  if (!obj || obj->kind != J_OBJ) return 0;
  for (i = 0; i < obj->len; i++) {
    if (strcmp(obj->pairs[i].key, key) == 0) return obj->pairs[i].val;
  }
  return 0;
}

static const JNode *query_path(const JNode *root, const char *dotpath)
{
  char key[64];
  const char *pcur;
  int ki;
  const JNode *cur;
  pcur = dotpath;
  ki = 0;
  cur = root;
  while (1) {
    if (*pcur == '.' || *pcur == '\0') {
      key[ki] = '\0';
      cur = find_key_in_object(cur, key);
      if (!cur) return 0;
      if (*pcur == '\0') return cur;
      pcur++;
      ki = 0;
      continue;
    }
    if (ki < (int)sizeof(key)-1) key[ki++] = *pcur;
    pcur++;
  }
}

int main(void)
{
  /* Use the parser's global 'p' and parse_value() to parse JSON, then query. */
  p = "{\"a\": {\"b\": {\"c\": 123}}, \"msg\": \"hi\"}";
  {
    const JNode *root = parse_value();
    const JNode *v = query_path(root, "a.b.c");
    if (v && v->kind == J_NUM) {
      printf("a.b.c=%g\n", v->num);
    }
    v = query_path(root, "msg");
    if (v && v->kind == J_STR) {
      printf("msg=%s\n", v->str);
    }
  }
  return 0;
}




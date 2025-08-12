/* C89: tiny JSON AST for a very small subset (objects with string keys, numbers, strings, arrays) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

typedef enum { J_NULL, J_NUM, J_STR, J_ARR, J_OBJ } JKind;

typedef struct JNode JNode;
typedef struct JPair JPair;

struct JNode {
  JKind kind;
  char *str;      /* for strings */
  double num;     /* for numbers */
  JNode **elems;  /* array elements */
  int len;
  JPair *pairs;   /* object pairs */
};

struct JPair {
  char *key;
  JNode *val;
};

static const char *p;

static void skip(void) { while (*p==' '||*p=='\n'||*p=='\r'||*p=='\t') p++; }

static char *dup_range(const char *a, const char *b)
{
  int n = (int)(b - a);
  char *s = (char*)malloc((unsigned)n + 1);
  if (!s) return 0;
  memcpy(s, a, (size_t)n);
  s[n] = '\0';
  return s;
}

static JNode *parse_value(void);

static JNode *make_node(JKind k)
{
  JNode *n = (JNode*)calloc(1, sizeof(JNode));
  if (n) n->kind = k;
  return n;
}

static JNode *parse_string(void)
{
  const char *start;
  if (*p != '"') { return 0; }
  p++;
  start = p;
  while (*p && *p != '"') { if (*p == '\\' && p[1]) p++; p++; }
  if (*p != '"') return 0;
  {
    JNode *n = make_node(J_STR);
    n->str = dup_range(start, p);
    p++;
    return n;
  }
}

static JNode *parse_number(void)
{
  const char *start = p;
  while (*p=='-'||*p=='+'||*p=='.'||isdigit((unsigned char)*p)) p++;
  {
    JNode *n = make_node(J_NUM);
    n->num = atof(start);
    return n;
  }
}

static JNode *parse_array(void)
{
  JNode *n;
  int cap = 4;
  if (*p != '[') { return 0; }
  p++;
  n = make_node(J_ARR);
  n->elems = (JNode**)malloc(sizeof(JNode*) * (size_t)cap);
  n->len = 0;
  skip();
  if (*p == ']') { p++; return n; }
  for (;;) {
    JNode *v;
    if (n->len >= cap) {
      JNode **tmp;
      cap *= 2;
      tmp = (JNode**)realloc(n->elems, sizeof(JNode*) * (size_t)cap);
      if (!tmp) return n;
      n->elems = tmp;
    }
    v = parse_value();
    n->elems[n->len++] = v;
    skip();
    if (*p == ',') { p++; skip(); continue; }
    if (*p == ']') { p++; break; }
    break;
  }
  return n;
}

static JNode *parse_object(void)
{
  JNode *n;
  int cap = 4;
  if (*p != '{') { return 0; }
  p++;
  n = make_node(J_OBJ);
  n->pairs = (JPair*)malloc(sizeof(JPair) * (size_t)cap);
  n->len = 0;
  skip();
  if (*p == '}') { p++; return n; }
  for (;;) {
    JNode *k; JNode *v;
    if (n->len >= cap) {
      JPair *tmp;
      cap *= 2;
      tmp = (JPair*)realloc(n->pairs, sizeof(JPair) * (size_t)cap);
      if (!tmp) return n;
      n->pairs = tmp;
    }
    k = parse_string(); if (!k) return n;
    skip(); if (*p != ':') return n; p++; skip();
    v = parse_value();
    n->pairs[n->len].key = k->str; n->pairs[n->len].val = v; n->len++;
    skip();
    if (*p == ',') { p++; skip(); continue; }
    if (*p == '}') { p++; break; }
    break;
  }
  return n;
}

static JNode *parse_value(void)
{
  skip();
  if (*p == '"') return parse_string();
  if (*p == '[') return parse_array();
  if (*p == '{') return parse_object();
  return parse_number();
}

static void print_node(const JNode *n, int depth)
{
  int i;
  for (i=0;i<depth;i++) printf("  ");
  switch (n->kind) {
    case J_STR: printf("STR \"%s\"\n", n->str); break;
    case J_NUM: printf("NUM %g\n", n->num); break;
    case J_ARR:
      printf("ARR[%d]\n", n->len);
      for (i=0;i<n->len;i++) print_node(n->elems[i], depth+1);
      break;
    case J_OBJ:
      printf("OBJ{%d}\n", n->len);
      for (i=0;i<n->len;i++) {
        int j; for (j=0;j<depth+1;j++) printf("  ");
        printf("%s:\n", n->pairs[i].key);
        print_node(n->pairs[i].val, depth+2);
      }
      break;
    default: printf("NULL\n"); break;
  }
}

int main(void)
{
  p = "{\"a\": [1, 2, 3], \"b\": {\"c\": 42}}";
  print_node(parse_value(), 0);
  return 0;
}



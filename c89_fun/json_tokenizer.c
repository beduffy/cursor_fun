/* C89: tiny JSON tokenizer (strings, numbers, punctuation) */
#include <stdio.h>
#include <ctype.h>

enum Tok { T_LBRACE, T_RBRACE, T_LBRACK, T_RBRACK, T_COLON, T_COMMA, T_STRING, T_NUMBER, T_END, T_ERR };

static const char *src;

static void skip_ws(void) { while (*src==' '||*src=='\n'||*src=='\r'||*src=='\t') src++; }

static enum Tok next_token(char *buf, int buflen)
{
  skip_ws();
  if (*src == '\0') return T_END;
  switch (*src) {
    case '{': src++; return T_LBRACE;
    case '}': src++; return T_RBRACE;
    case '[': src++; return T_LBRACK;
    case ']': src++; return T_RBRACK;
    case ':': src++; return T_COLON;
    case ',': src++; return T_COMMA;
    case '"': {
      int i = 0; src++; /* skip opening quote */
      while (*src && *src != '"' && i < buflen-1) {
        if (*src == '\\') {
          src++;
          if (*src) { buf[i++] = *src++; }
          else { return T_ERR; }
        } else {
          buf[i++] = *src++;
        }
      }
      if (*src != '"') { return T_ERR; }
      src++;
      buf[i] = 0;
      return T_STRING;
    }
    default:
      if (*src=='-' || isdigit((unsigned char)*src)) {
        int i = 0;
        if (*src=='-') { buf[i++]=*src++; }
        while (isdigit((unsigned char)*src) && i < buflen-1) buf[i++]=*src++;
        if (*src=='.' && i < buflen-1) { buf[i++]=*src++; while (isdigit((unsigned char)*src) && i < buflen-1) buf[i++]=*src++; }
        buf[i]=0; return T_NUMBER;
      }
  }
  return T_ERR;
}

int main(void)
{
  char buf[64]; enum Tok t;
  src = "{\"a\": 1, \"b\": [2, 3]}";
  for (;;) {
    t = next_token(buf, 64);
    if (t == T_END) break;
    if (t == T_ERR) { printf("ERR\n"); break; }
    switch (t) {
      case T_LBRACE: printf("{\n"); break;
      case T_RBRACE: printf("}\n"); break;
      case T_LBRACK: printf("[\n"); break;
      case T_RBRACK: printf("]\n"); break;
      case T_COLON: printf(":\n"); break;
      case T_COMMA: printf(",\n"); break;
      case T_STRING: printf("STRING(%s)\n", buf); break;
      case T_NUMBER: printf("NUMBER(%s)\n", buf); break;
      default: break;
    }
  }
  return 0;
}



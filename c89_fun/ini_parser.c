/* C89: tiny INI parser (sections and key=value, no escapes) */
#include <stdio.h>
#include <string.h>

static void trim(char *s)
{
  char *e;
  while (*s==' '||*s=='\t') s++;
  e = s + (int)strlen(s);
  while (e > s && (e[-1]==' '||e[-1]=='\t'||e[-1]=='\r'||e[-1]=='\n')) { e--; }
  *e = '\0';
}


int main(void)
{
  const char *ini = "[core]\nname=demo\ncount=3\n\n[paths]\nroot=/tmp\n";
  char buf[128];
  char section[64];
  const char *p = ini;
  section[0] = '\0';

  while (*p) {
    int i = 0;
    while (p[i] && p[i] != '\n' && i < (int)sizeof(buf)-1) { buf[i] = p[i]; i++; }
    buf[i] = '\0';
    p += (p[i] == '\n') ? (i + 1) : i;

    if (buf[0] == ';' || buf[0] == '#' || buf[0] == '\0') continue;
    if (buf[0] == '[') {
      char *r = strchr(buf, ']');
      if (r) { *r = '\0'; strncpy(section, buf + 1, sizeof(section)-1); section[sizeof(section)-1]='\0'; }
      continue;
    } else {
      char *eq = strchr(buf, '=');
      if (eq) {
        *eq = '\0';
        trim(buf); trim(eq + 1);
        printf("%s.%s=%s\n", section, buf, eq + 1);
      }
    }
  }

  return 0;
}



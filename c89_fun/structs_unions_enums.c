/* C89 structs, unions, enums */
#include <stdio.h>

struct Point { int x; int y; };

union IntOrChar {
  int i;
  char c[sizeof(int)];
};

enum Color { COLOR_RED = 1, COLOR_GREEN = 2, COLOR_BLUE = 4 };

int main(void)
{
  struct Point p;
  union IntOrChar u;
  enum Color color;

  p.x = 3; p.y = 4;
  printf("Point: (%d,%d)\\n", p.x, p.y);

  u.i = 0x41424344; /* 'A','B','C','D' in some order depending on endianness */
  printf("union as int: 0x%X, bytes: %d %d %d %d\\n",
         u.i, (int)u.c[0], (int)u.c[1], (int)u.c[2], (int)u.c[3]);

  color = COLOR_GREEN;
  printf("Color value: %d\\n", (int)color);

  return 0;
}



/* C89: simple argv parser demo (-n <num>, --name <str>, -v) */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct Options { int verbose; int number; const char *name; };

static void parse_args(int argc, char **argv, struct Options *opt)
{
  int i = 1; opt->verbose = 0; opt->number = 0; opt->name = "";
  while (i < argc) {
    if (strcmp(argv[i], "-v") == 0) { opt->verbose = 1; i++; }
    else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) { opt->number = atoi(argv[i+1]); i += 2; }
    else if (strcmp(argv[i], "--name") == 0 && i + 1 < argc) { opt->name = argv[i+1]; i += 2; }
    else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printf("usage: %s [-v] -n <num> --name <str>\n", argv[0]); return;
    } else { i++; }
  }
}


int main(int argc, char **argv)
{
  struct Options opt;
  if (argc <= 1) {
    /* demo */
    char *demo_argv[] = { "argparse_demo", "-v", "-n", "7", "--name", "ben", 0 };
    parse_args(6, demo_argv, &opt);
  } else {
    parse_args(argc, argv, &opt);
  }
  printf("verbose=%d number=%d name=%s\n", opt.verbose, opt.number, opt.name);
  return 0;
}




/* C89: simple logger with levels (uses getenv) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { LOG_TRACE = 0, LOG_INFO = 1, LOG_WARN = 2, LOG_ERROR = 3 };

static int global_log_level = LOG_INFO;

static void logger_init(void)
{
  const char *lvl = getenv("LOG_LEVEL");
  if (lvl) {
    if (strcmp(lvl, "TRACE") == 0) global_log_level = LOG_TRACE;
    else if (strcmp(lvl, "INFO") == 0) global_log_level = LOG_INFO;
    else if (strcmp(lvl, "WARN") == 0) global_log_level = LOG_WARN;
    else if (strcmp(lvl, "ERROR") == 0) global_log_level = LOG_ERROR;
  }
}


static void log_msg(int level, const char *msg)
{
  if (level < global_log_level) return;
  switch (level) {
    case LOG_TRACE: fprintf(stderr, "[TRACE] %s\n", msg); break;
    case LOG_INFO:  fprintf(stderr, "[INFO ] %s\n", msg); break;
    case LOG_WARN:  fprintf(stderr, "[WARN ] %s\n", msg); break;
    case LOG_ERROR: fprintf(stderr, "[ERROR] %s\n", msg); break;
    default: break;
  }
}


int main(void)
{
  logger_init();
  log_msg(LOG_TRACE, "trace hidden by default");
  log_msg(LOG_INFO,  "hello info");
  log_msg(LOG_WARN,  "something mild");
  log_msg(LOG_ERROR, "something bad");
  return 0;
}




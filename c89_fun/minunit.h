/* Tiny C89-friendly unit test helpers */
#ifndef MINUNIT_H_INCLUDED
#define MINUNIT_H_INCLUDED

#define MU_ASSERT(msg, test) do { if (!(test)) return msg; } while (0)
#define MU_RUN_TEST(test) do { const char *mu_msg = test(); tests_run++; \
                               if (mu_msg) return mu_msg; } while (0)

extern int tests_run;

#endif




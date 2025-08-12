/* C89: micro unit tests for dynamic_array.c via minunit */
#include <stdio.h>
#include <stdlib.h>

#define MINUNIT_H_IMPLEMENTATION 1
#include "minunit.h"

int tests_run = 0;

/* Include the implementation so we test internal (static) functions in same TU.
   Rename its demo main() to avoid a duplicate symbol. */
#define main dynamic_array_demo_main
#include "dynamic_array.c"
#undef main

static const char * test_push_and_size(void)
{
  struct IntVec v; unsigned long i;
  vec_init(&v);
  for (i = 0; i < 5; i++) vec_push(&v, (int)i);
  MU_ASSERT("size should be 5", v.size == 5);
  vec_free(&v);
  return 0;
}

static const char * all_tests(void)
{
  MU_RUN_TEST(test_push_and_size);
  return 0;
}

int main(void)
{
  const char *result = all_tests();
  if (result != 0) {
    printf("TEST FAIL: %s\n", result);
  } else {
    printf("ALL TESTS PASSED\n");
  }
  printf("Tests run: %d\n", tests_run);
  return 0;
}



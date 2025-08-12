/* C89: simple fixed-size block memory pool (free-list) */
#include <stdio.h>
#include <stdlib.h>

#define POOL_BLOCK_SIZE 32
#define POOL_BLOCK_COUNT 16

struct PoolNode { struct PoolNode *next; };

struct MemoryPool {
  unsigned char storage[POOL_BLOCK_SIZE * POOL_BLOCK_COUNT];
  struct PoolNode *free_list;
};

static void pool_init(struct MemoryPool *pool)
{
  unsigned int i;
  unsigned char *base = pool->storage;
  pool->free_list = (struct PoolNode*)base;
  for (i = 0; i < POOL_BLOCK_COUNT - 1; i++) {
    struct PoolNode *node = (struct PoolNode*)(base + i * POOL_BLOCK_SIZE);
    node->next = (struct PoolNode*)(base + (i + 1) * POOL_BLOCK_SIZE);
  }
  ((struct PoolNode*)(base + (POOL_BLOCK_COUNT - 1) * POOL_BLOCK_SIZE))->next = 0;
}

static void *pool_alloc(struct MemoryPool *pool)
{
  struct PoolNode *node = pool->free_list;
  if (node == 0) return 0;
  pool->free_list = node->next;
  return (void*)node;
}

static void pool_free(struct MemoryPool *pool, void *ptr)
{
  struct PoolNode *node = (struct PoolNode*)ptr;
  node->next = pool->free_list;
  pool->free_list = node;
}

int main(void)
{
  struct MemoryPool pool;
  void *p[POOL_BLOCK_COUNT];
  unsigned int i;

  pool_init(&pool);
  for (i = 0; i < POOL_BLOCK_COUNT; i++) {
    p[i] = pool_alloc(&pool);
    if (!p[i]) {
      printf("alloc failed at %u\n", i);
      return 0;
    }
  }
  /* Pool exhausted now */
  if (pool_alloc(&pool) == 0) {
    printf("pool exhausted as expected\n");
  }
  /* Free some and allocate again */
  for (i = 0; i < POOL_BLOCK_COUNT; i += 2) {
    pool_free(&pool, p[i]);
  }
  for (i = 0; i < 3; i++) {
    void *q = pool_alloc(&pool);
    printf("re-alloc %u: %p\n", i, q);
  }
  return 0;
}




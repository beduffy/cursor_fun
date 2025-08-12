/* C89: simple open-addressing hash table with linear probing for int->int */
#include <stdio.h>
#include <stdlib.h>

#define TABLE_SIZE 101 /* a small prime */

struct Entry {
  int key;
  int value;
  int used; /* 0=empty, 1=occupied, 2=deleted (tombstone) */
};

struct HashTable {
  struct Entry entries[TABLE_SIZE];
};

static unsigned int hash_int(int x)
{
  /* Knuth multiplicative hash (mod 2^32), then reduce */
  unsigned long u = (unsigned long)(unsigned int)x;
  u *= 2654435761ul;
  return (unsigned int)(u % (unsigned long)TABLE_SIZE);
}

static void ht_init(struct HashTable *ht)
{
  unsigned int i;
  for (i = 0; i < TABLE_SIZE; i++) {
    ht->entries[i].used = 0;
  }
}

static int ht_put(struct HashTable *ht, int key, int value)
{
  unsigned int idx = hash_int(key);
  unsigned int i;
  for (i = 0; i < TABLE_SIZE; i++) {
    unsigned int j = (idx + i) % TABLE_SIZE;
    if (ht->entries[j].used == 0 || ht->entries[j].used == 2) {
      ht->entries[j].key = key;
      ht->entries[j].value = value;
      ht->entries[j].used = 1;
      return 1;
    }
    if (ht->entries[j].used == 1 && ht->entries[j].key == key) {
      ht->entries[j].value = value; /* update */
      return 1;
    }
  }
  return 0; /* table full */
}

static int ht_get(const struct HashTable *ht, int key, int *out)
{
  unsigned int idx = hash_int(key);
  unsigned int i;
  for (i = 0; i < TABLE_SIZE; i++) {
    unsigned int j = (idx + i) % TABLE_SIZE;
    if (ht->entries[j].used == 0) return 0; /* not found */
    if (ht->entries[j].used == 1 && ht->entries[j].key == key) {
      *out = ht->entries[j].value;
      return 1;
    }
  }
  return 0;
}

static int ht_del(struct HashTable *ht, int key)
{
  unsigned int idx = hash_int(key);
  unsigned int i;
  for (i = 0; i < TABLE_SIZE; i++) {
    unsigned int j = (idx + i) % TABLE_SIZE;
    if (ht->entries[j].used == 0) return 0; /* not found */
    if (ht->entries[j].used == 1 && ht->entries[j].key == key) {
      ht->entries[j].used = 2; /* tombstone */
      return 1;
    }
  }
  return 0;
}

int main(void)
{
  struct HashTable ht;
  int out;
  ht_init(&ht);
  ht_put(&ht, 10, 100);
  ht_put(&ht, 20, 200);
  ht_put(&ht, 30, 300);
  if (ht_get(&ht, 20, &out)) printf("20->%d\n", out);
  ht_del(&ht, 20);
  if (!ht_get(&ht, 20, &out)) printf("20 not found (deleted)\n");
  return 0;
}



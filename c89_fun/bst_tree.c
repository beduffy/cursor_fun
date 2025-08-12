/* C89: basic binary search tree (int keys), insert/search/inorder */
#include <stdio.h>
#include <stdlib.h>

struct Node { int key; struct Node *l, *r; };

static struct Node *make(int key)
{
  struct Node *n = (struct Node*)malloc(sizeof *n);
  if (!n) {
    return 0;
  }
  n->key = key;
  n->l = 0;
  n->r = 0;
  return n;
}

static struct Node *ins(struct Node *root, int key)
{
  if (!root) return make(key);
  if (key < root->key) root->l = ins(root->l, key);
  else if (key > root->key) root->r = ins(root->r, key);
  return root;
}

static int find(struct Node *root, int key)
{
  while (root) {
    if (key == root->key) return 1;
    root = key < root->key ? root->l : root->r;
  }
  return 0;
}

static void inorder(struct Node *root)
{
  if (!root) return;
  inorder(root->l); printf("%d ", root->key); inorder(root->r);
}

static void freetree(struct Node *root)
{
  if (!root) {
    return;
  }
  freetree(root->l);
  freetree(root->r);
  free(root);
}

int main(void)
{
  struct Node *root = 0;
  int keys[7] = { 5, 2, 8, 1, 3, 7, 9 };
  int i;
  for (i = 0; i < 7; i++) root = ins(root, keys[i]);
  inorder(root); printf("\nfind 3? %d, find 6? %d\n", find(root,3), find(root,6));
  freetree(root);
  return 0;
}



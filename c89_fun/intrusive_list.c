/* C89: intrusive singly linked list (no heap allocation here) */
#include <stdio.h>

struct Node {
  int value;
  struct Node *next;
};

static void push_front(struct Node **head, struct Node *n)
{
  n->next = *head;
  *head = n;
}

static void print_list(const struct Node *head)
{
  const struct Node *it = head;
  while (it) {
    printf("%d ", it->value);
    it = it->next;
  }
  printf("\n");
}

int main(void)
{
  struct Node a, b, c;
  struct Node *head = 0;
  a.value = 1; b.value = 2; c.value = 3;
  a.next = 0; b.next = 0; c.next = 0;
  push_front(&head, &c);
  push_front(&head, &b);
  push_front(&head, &a);
  print_list(head);
  return 0;
}




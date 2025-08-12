/* C89: simple job queue with function pointer tasks */
#include <stdio.h>

typedef void (*JobFn)(void *ctx);

struct Job { JobFn fn; void *ctx; };

struct Queue { struct Job jobs[32]; int head; int tail; };

static void q_init(struct Queue *q){ q->head=0; q->tail=0; }
static int  q_push(struct Queue *q, JobFn fn, void *ctx){ int nt=(q->tail+1)&31; if(nt==q->head) return 0; q->jobs[q->tail].fn=fn; q->jobs[q->tail].ctx=ctx; q->tail=nt; return 1; }
static int  q_pop(struct Queue *q, struct Job *out){ if(q->head==q->tail) return 0; *out=q->jobs[q->head]; q->head=(q->head+1)&31; return 1; }

static void print_job(void *ctx){ printf("job: %s\n", (const char*)ctx); }
static void count_job(void *ctx){ int i; int *n=(int*)ctx; for(i=0;i<*n;i++){} printf("counted %d\n", *n); }

int main(void)
{
  struct Queue q; struct Job j; int n=100000;
  q_init(&q);
  q_push(&q, print_job, (void*)"hello");
  q_push(&q, count_job, (void*)&n);
  q_push(&q, print_job, (void*)"done");
  while(q_pop(&q, &j)) j.fn(j.ctx);
  return 0;
}





#include <stdlib.h>

#include "buffer.c"

#include <sys/time.h>
#include <stdio.h>

int main () {
  buf_init(400);

  long i;
  struct record r;
  struct timeval t;

  for (i = 0; i < 400; ++i) {
    gettimeofday(&t, NULL);
    r.ms = t.tv_usec;
    r.val = i;
    buf_add(&r);
  }

  struct record* b;
  _buf(&b, NULL, NULL, 0);
  for (i = 0; i < 400; ++i) {
    printf("%ld, %d\n", b[i].ms, b[i].val);
  }
}



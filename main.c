#include <stdio.h>
#include <stdlib.h>

#define ALIGN 16
#define PUSH_ALIGN(offset, align) (offset + (align - 1)) & (~align)

//====================
//
//    Arena
//
//====================

typedef struct {
  char *base;
  size_t offset;
  size_t capacity;
} Arena;

Arena arena_create(size_t capacity) {
  Arena a = {0};
  a.capacity = capacity;
  a.base = malloc(capacity);
  return a;
}

void arena_destroy(Arena *a) { free(a->base); }

char *arena_alloc(Arena *a, size_t size) {
  size_t aligned_offset = PUSH_ALIGN(a->offset, ALIGN);
  if (aligned_offset + size > a->capacity) {
    return NULL;
  }
  char *addr = a->base + aligned_offset;
  a->offset = aligned_offset + size;
  return addr;
}

//====================
//
//    ODE
//
//====================

typedef void ode_fun_t(double t, double *y, double *dydx, void *ctx);
typedef void ode_stepper(double t, double dt, double *y, double *dydx,
                         ode_fun_t fun, size_t dim_y, void *ctx,
                         double *scratch);

void sum_arrays(double *out, double *a, double *b, size_t num_elems) {
  for (size_t i = 0; i < num_elems; i++) {
    out[i] = a[i] + b[i];
  }
}
void scalar_prod(double *out, double *a, double k, size_t num_elems) {
  for (size_t i = 0; i < num_elems; i++) {
    out[i] = a[i] * k;
  }
}

void euler_step(double t, double dt, double *y, double *dydx, ode_fun_t fun,
                size_t dim_y, void *ctx, double *scratch) {
  fun(t, y, dydx, ctx);
  for (size_t i = 0; i < dim_y; i++) {
    y[i] += dt * dydx[i];
  }
}

void rk4_step(double t, double dt, double *y, double *dydx, ode_fun_t fun,
              size_t dim_y, void *ctx, double *scratch) {

  double *k1 = scratch;
  double *k2 = scratch + dim_y;
  double *k3 = scratch + 2 * dim_y;
  double *k4 = scratch + 3 * dim_y;
  double *ytmp = scratch + 4 * dim_y;

  fun(t, y, k1, ctx);

  scalar_prod(ytmp, k1, dt / 2, dim_y);
  sum_arrays(ytmp, y, ytmp, dim_y);
  fun(t + dt / 2, ytmp, k2, ctx);

  scalar_prod(ytmp, k2, dt / 2, dim_y);
  sum_arrays(ytmp, y, ytmp, dim_y);
  fun(t + dt / 2, ytmp, k3, ctx);

  scalar_prod(ytmp, k3, dt, dim_y);
  sum_arrays(ytmp, y, ytmp, dim_y);
  fun(t + dt, ytmp, k4, ctx);
  for (size_t i = 0; i < dim_y; i++) {
    y[i] += dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
}

void simple_exp(double t, double *y, double *dydx, void *ctx) {
  dydx[0] = y[0];
}

int main() {
  printf("Hi, mom\n");

  size_t N = 1000;

  double dydx[1];
  double y[1];
  y[0] = 1;
  double dt = 1.0 / (N + 1);
  double t = 0;
  double scratch[5];
  for (size_t i = 0; i < N; i += 1) {
    printf("%f\t%.3f\n", t, y[0]);
    rk4_step(t, dt, y, dydx, simple_exp, 1, NULL, scratch);
    t += dt;
  }

  return 0;
}

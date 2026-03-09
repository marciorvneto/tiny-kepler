#ifndef TINY_KEPLER_H
#define TINY_KEPLER_H
#include <math.h>
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

Arena arena_create(size_t capacity);

void arena_destroy(Arena *a);

char *arena_alloc(Arena *a, size_t size);

//====================
//
//    ODE
//
//====================

typedef void ode_fun_t(double t, double *y, double *dydt, void *ctx);
typedef void ode_stepper_t(double t, double dt, double *y, double *dydt,
                           ode_fun_t fun, size_t dim_y, void *ctx,
                           double *scratch);

void sum_arrays(double *out, double *a, double *b, size_t num_elems);
void scalar_prod(double *out, double *a, double k, size_t num_elems);

void euler_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
                void *ctx, double *scratch);

void rk4_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
              void *ctx, double *scratch);

// accel_fn: only computes acceleration from position
typedef void accel_fn_t(double *pos, double *accel, void *ctx);
void verlet_step(double t, double dt, double *y, accel_fn_t accel,
                 size_t n_dimensions, void *ctx, double *scratch);

#ifdef TINY_KEPLER_IMPLEMENTATION

//====================
//
//    Arena
//
//====================

Arena arena_create(size_t capacity) {
  Arena a = {0};
  a.capacity = capacity;
  a.base = (char *)malloc(capacity);
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

void euler_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
                void *ctx, double *scratch) {
  fun(t, y, scratch, ctx);
  for (size_t i = 0; i < dim_y; i++) {
    y[i] += dt * scratch[i];
  }
}

void rk4_step(double t, double dt, double *y, ode_fun_t fun, size_t dim_y,
              void *ctx, double *scratch) {

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

// accel_fn: only computes acceleration from position
void verlet_step(double t, double dt, double *y, accel_fn_t accel,
                 size_t n_dimensions, void *ctx, double *scratch) {
  double *pos = y;
  double *vel = y + n_dimensions;
  double *a = scratch;

  // Fist half-kick
  accel(pos, a, ctx);
  for (size_t i = 0; i < n_dimensions; i++) {
    vel[i] += 0.5 * a[i] * dt;
  }

  // Drift
  for (size_t i = 0; i < n_dimensions; i++) {
    pos[i] += vel[i] * dt;
  }

  // Second half-kick
  accel(pos, a, ctx);
  for (size_t i = 0; i < n_dimensions; i++) {
    vel[i] += 0.5 * a[i] * dt;
  }
}

#endif

#endif // TINY_KEPLER_H

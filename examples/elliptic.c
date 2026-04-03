#define TINY_KEPLER_IMPLEMENTATION
#include "string.h"
#include "tiny-kepler.h"

typedef struct {
  double mu;
} GravParams;

// Regular definition
void gravity(double t, double *y, double *dydt, void *ctx) {
  // a = -mu/r³ * r

  GravParams *p = ctx;
  double x = y[0];
  double y_ = y[1];
  double vx = y[2];
  double vy = y[3];

  double r3 = pow(x * x + y_ * y_, 1.5); // r³ = (r²)^3/2

  dydt[0] = vx;
  dydt[1] = vy;
  dydt[2] = -p->mu / r3 * x;
  dydt[3] = -p->mu / r3 * y_;
}

// For Verlet
void gravity_accel(double *pos, double *accel, void *ctx) {
  GravParams *p = ctx;
  double x = pos[0];
  double y = pos[1];

  double r3 = pow(x * x + y * y, 1.5); // r³ = (r²)^3/2

  accel[0] = -p->mu / r3 * x;
  accel[1] = -p->mu / r3 * y;
}

double norm_two_entries(double *pos) {
  return sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
}

int main(int argc, char **argv) {
  const char *method = (argc > 1) ? argv[1] : "rk4";

  GravParams params = {.mu = 1.0};
  double y[4] = {0.5, 0.0, 0.0, 2.5};

  int orbits = 1;
  int steps_per_orbit = 1000;
  int steps = orbits * steps_per_orbit;
  double period = 0.3 * 2 * M_PI;
  double dt = period / steps_per_orbit;
  double t = 0;
  double scratch[20];

  for (size_t i = 0; i < steps; i++) {
    double r = norm_two_entries(y);
    double v = norm_two_entries(y + 2);
    double energy = 0.5 * v * v - params.mu / r;
    printf("%.4f\t%.6f\t%.6f\t%.6f\t%.10f\n", t, y[0], y[1], r, energy);

    if (strcmp(method, "euler") == 0)
      euler_step(t, dt, y, gravity, 4, &params, scratch);
    else if (strcmp(method, "verlet") == 0)
      verlet_step(t, dt, y, gravity_accel, 2, &params, scratch);
    else
      rk4_step(t, dt, y, gravity, 4, &params, scratch);

    t += dt;
  }
  return 0;
}

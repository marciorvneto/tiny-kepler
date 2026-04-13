#define TINY_KEPLER_IMPLEMENTATION
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include "tiny-kepler.h"
#include "tinyla.h"

#define NUM_NODES 20
//#define NUM_ENTTS 5 // Spacecraft, Sun, Earth, Moon, Mars
#define NUM_ENTTS 6 // Spacecraft, Sun, Earth, Moon, Mars, Venus
void n_body_dump_to_file(NBodyIntegrationContext *ctx, const char *path) {
    FILE *f = fopen(path, "wb");
    if (!f) return;

    NBodyScenario *s = ctx->scenario;
    double current_t = 0;
    double dt = 0.0001; // Integration step
    
    // Header info for the visualizer
    uint32_t n_entities = (uint32_t)s->num_entities;
    fwrite(&n_entities, sizeof(uint32_t), 1, f);

    // Initial State copy
    memcpy(ctx->t_x, s->x, s->num_entities * sizeof(double));
    memcpy(ctx->t_y, s->y, s->num_entities * sizeof(double));
    memcpy(ctx->t_z, s->z, s->num_entities * sizeof(double));
    memcpy(ctx->t_vx, s->vx, s->num_entities * sizeof(double));
    memcpy(ctx->t_vy, s->vy, s->num_entities * sizeof(double));
    memcpy(ctx->t_vz, s->vz, s->num_entities * sizeof(double));

    n_body_integration_state_to_vec(ctx->y, ctx->Phi, s->num_entities, 
                                   ctx->t_x, ctx->t_y, ctx->t_z, 
                                   ctx->t_vx, ctx->t_vy, ctx->t_vz);

    // Run through the mission nodes
		for (size_t i = 0; i < s->num_nodes; i++) {
        while (current_t < s->t[i]) {
            float t_float = (float)current_t;
            fwrite(&t_float, sizeof(float), 1, f);
            for (size_t j = 0; j < s->num_entities; j++) {
                float x = (float)ctx->t_x[j];
                float y = (float)ctx->t_y[j];
                fwrite(&x, sizeof(float), 1, f);
                fwrite(&y, sizeof(float), 1, f);
            }

            rk4_step(current_t, dt, ctx->y, n_body_ode_fn, ctx->dim_y, ctx, ctx->scratch);
            
            n_body_vec_to_integration_state(ctx->y, ctx->Phi, s->num_entities, 
                                            ctx->t_x, ctx->t_y, ctx->t_z, 
                                            ctx->t_vx, ctx->t_vy, ctx->t_vz);
            current_t += dt;
        }

        ctx->y[3] += s->dv_x[i];
        ctx->y[4] += s->dv_y[i];
        ctx->y[5] += s->dv_z[i];

        n_body_vec_to_integration_state(ctx->y, ctx->Phi, s->num_entities, 
                                        ctx->t_x, ctx->t_y, ctx->t_z, 
                                        ctx->t_vx, ctx->t_vy, ctx->t_vz);
    }
    fclose(f);
}


int main(int argc, char **argv) {

		char *output_path = "./n-body-results.out";
		if(argc > 1){
			output_path = argv[1];
		}

    Arena a = arena_create(10 * 1024 * 1024); // 1MB
    tla_Arena la = tla_arena_create(10 * 1024 * 1024);

    // ---- Scenario definition ----

    NBodyScenario scenario = {0};
    scenario.num_nodes = NUM_NODES;
    scenario.num_entities = NUM_ENTTS;

    scenario.mass = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.x    = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.y    = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.z    = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    
    scenario.vx   = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.vy   = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.vz   = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.ax   = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.ay   = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    scenario.az   = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));

    scenario.t    = (double*) arena_alloc(&a, NUM_NODES * sizeof(double));
    scenario.dv_x = (double*) arena_alloc(&a, NUM_NODES * sizeof(double));
    scenario.dv_y = (double*) arena_alloc(&a, NUM_NODES * sizeof(double));
    scenario.dv_z = (double*) arena_alloc(&a, NUM_NODES * sizeof(double));

    // 1. Sun (Origin)
    scenario.mass[1] = 1.9885e30; 
    scenario.x[1]  = 0.0; scenario.y[1]  = 0.0; scenario.z[1]  = 0.0;
    scenario.vx[1] = 0.0; scenario.vy[1] = 0.0; scenario.vz[1] = 0.0;

    // 2. Earth (1 AU from Sun)
    scenario.mass[2] = 5.972e24;
    scenario.x[2]  = 149.598e6; scenario.y[2]  = 0.0;   scenario.z[2]  = 0.0;
    scenario.vx[2] = 0.0;       scenario.vy[2] = 29.78; scenario.vz[2] = 0.0;

    scenario.mass[3] = 7.342e22;
    scenario.x[3]  = scenario.x[2] + 384400.0; 
    scenario.y[3]  = scenario.y[2];
    scenario.z[3]  = 0.0;
    scenario.vx[3] = scenario.vx[2]; 
    scenario.vy[3] = scenario.vy[2] + 1.022; // Earth vel + Moon orbital vel
    scenario.vz[3] = 0.0;

    // 4. Mars (~1.52 AU from Sun)
    scenario.mass[4] = 6.4171e23;
    scenario.x[4]  = 227.939e6; scenario.y[4]  = 0.0;   scenario.z[4]  = 0.0;
    scenario.vx[4] = 0.0;       scenario.vy[4] = 24.07; scenario.vz[4] = 0.0;

    // 5. Venus (~0.72 AU from Sun)
    scenario.mass[5] = 4.8675e24;
    scenario.x[5]  = 108.208e6; scenario.y[5]  = 0.0;   scenario.z[5]  = 0.0;
    scenario.vx[5] = 0.0;       scenario.vy[5] = 35.02; scenario.vz[5] = 0.0;

    // 0. Spacecraft (LEO around Earth at 200 km altitude)
    scenario.mass[0] = 10000.0;
    scenario.x[0]  = scenario.x[2] + 6578.0; 
    scenario.y[0]  = scenario.y[2];
    scenario.z[0]  = 0.0;
    scenario.vx[0] = scenario.vx[2]; 
    scenario.vy[0] = scenario.vy[2] + circular_orbital_velocity(398600.0, 6578.0); // Earth vel + LEO orbital vel
    scenario.vz[0] = 0.0;

		// Initial time
		double tmax = 10;
		for(size_t i = 0; i < scenario.num_nodes; i++){
			scenario.t[i] = tmax / scenario.num_nodes * i;
		}

		to_canonical_units(&scenario);

    // ---- Simulation ----

    double *t_x  = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_y  = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_z  = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_vx = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_vy = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_vz = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_ax = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_ay = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));
    double *t_az = (double*) arena_alloc(&a, NUM_ENTTS * sizeof(double));

		// State transition matrix (STM) -> 6x6
    tla_Matrix *Phi       = tla_matrix_eye(&la, 6);
    tla_Matrix *Phi_dot   = tla_matrix_eye(&la, 6);
    tla_Matrix *Phi_acumm = tla_matrix_eye(&la, 6);
    tla_Matrix *Phi_temp  = tla_matrix_eye(&la, 6);
    tla_Matrix **leg_Phis = (tla_Matrix**) arena_alloc(&a, NUM_NODES * sizeof(tla_Matrix*));
		for(size_t i = 0; i < NUM_NODES; i++){
			leg_Phis[i] = tla_matrix_of_shape(&la, Phi, 0);
		}

		// Gravity tensor (3x3)
    tla_Matrix *G = tla_matrix_eye(&la, 3);
		size_t dim_y  = 6 * scenario.num_entities + 36; // Entities + STM
																													//
		// Integrator
    double *scratch = (double*) arena_alloc(&a, 5 * dim_y * sizeof(double));
    double *y       = (double*) arena_alloc(&a, dim_y * sizeof(double));


		NBodyIntegrationContext ctx = {
			.scenario  = &scenario,
			.num_nodes = NUM_NODES,
			.G         = G,
			.Phi       = Phi,
			.Phi_dot   = Phi_dot,
			.Phi_acumm = Phi_acumm,
			.Phi_temp  = Phi_temp,
			.leg_Phis  = leg_Phis,
			.t_x       = t_x,
			.t_y       = t_y,
			.t_z       = t_z,
			.t_vx      = t_vx,
			.t_vy      = t_vy,
			.t_vz      = t_vz,
			.t_ax      = t_ax,
			.t_ay      = t_ay,
			.t_az      = t_az,
      .y         = y,
      .scratch   = scratch,
      .dim_y     = dim_y
		};

    // ---- Optimization ----


    size_t dim_x = NUM_NODES * 4;
    size_t dim_c = 3; // One constraint per coordinate (at least for now)


    tla_Matrix *H             = tla_matrix_of_value(&la, dim_x, dim_x, 0.0);
    tla_Matrix *Jc            = tla_matrix_of_value(&la, dim_c, dim_x, 0.0);
    tla_Vector *c             = tla_vector_of_value(&la, dim_c, 0.0);
    tla_Vector *grad_L        = tla_vector_of_value(&la, dim_c + dim_x, 0.0);
    tla_Vector *grad_f        = tla_vector_of_value(&la, dim_x, 0.0);

    tla_Vector *step          = tla_vector_of_value(&la, dim_x + dim_c, 0.0);
    tla_Vector *x_old         = tla_vector_of_value(&la, dim_x, 0.0);
    tla_Vector *grad_L_old    = tla_vector_of_value(&la, dim_c + dim_x, 0.0);

    tla_Matrix *KKT_matrix    = tla_matrix_of_value(&la, dim_c + dim_x, dim_c + dim_x, 0.0);
    tla_Vector *KKT_rhs       = tla_vector_of_value(&la, dim_c + dim_x, 0.0);
    tla_Matrix *KKT_augmented = tla_matrix_of_value(&la, dim_c + dim_x, dim_c + dim_x + 1, 0.0);

    SQPOptimizerCtx sqp_ctx = {
			.arena=&la,
      .dim_x=dim_x,
      .dim_c=dim_c,
      .H=H,         
      .Jc=Jc,        
      .c=c,         
      .grad_L=grad_L,    
      .grad_f=grad_f,    
      .step=step,      
      .x_old=x_old,     
      .grad_L_old=grad_L_old,
      .KKT_matrix=KKT_matrix,
      .KKT_rhs=KKT_rhs,   
      .KKT_augmented=KKT_augmented,   
			.physics_ctx=&ctx
    };

		double x[4 * NUM_NODES] = {0};
		double lagrange[3]      = {0};
		double tol              = 1e-4;

		double tmax_guess = 3.5;
    for(size_t i = 0; i < NUM_NODES; i++){
      x[4 * i + 0] = 0.0;
      x[4 * i + 1] = 0.0;
      x[4 * i + 2] = 0.0;
      // x[4 * i + 3] = 4.5 / NUM_NODES * (i + 1);
			// Idea: use a Chebyshev grid
      x[4 * i + 3] = tmax_guess / 2 * (1 - cos((i+1) * M_PI / (NUM_NODES - 1)));
    }
		x[0] = 0.00;
		x[1] = 0.25;
		x[2] = 0.00;

		optimize_sqp(n_body_evaluate_kkt_state, dim_x, x, lagrange, tol, &sqp_ctx);

		n_body_dump_to_file(&ctx, output_path);

    arena_destroy(&a);
    tla_arena_destroy(&la);
    return 0;
}

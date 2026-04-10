#define TINY_KEPLER_IMPLEMENTATION
#include <string.h>
#include <stddef.h>
#include "tiny-kepler.h"
#include "tinyla.h"

#define NUM_NODES 5
#define NUM_ENTTS 5 // Spacecraft, Sun, Earth, Moon, Mars

int main(int argc, char **argv) {

    Arena a = arena_create(1024 * 1024); // 1MB
    tla_Arena la = tla_arena_create(1024 * 1024);

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

    scenario.t   = (double*) arena_alloc(&a, NUM_NODES * sizeof(double));
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

    // 0. Spacecraft (LEO around Earth at 200 km altitude)
    scenario.mass[0] = 10000.0;
    scenario.x[0]  = scenario.x[2] + 6578.0; 
    scenario.y[0]  = scenario.y[2];
    scenario.z[0]  = 0.0;
    scenario.vx[0] = scenario.vx[2]; 
    scenario.vy[0] = scenario.vy[2] + circular_orbital_velocity(398600.0, 6578.0); // Earth vel + LEO orbital vel
    scenario.vz[0] = 0.0;

		// Initial time
		double tmax      = 10;
		for(size_t i = 0; i < scenario.num_nodes; i++){
			scenario.t[i] = tmax / scenario.num_nodes * i;
		}

		to_canonical_units(&scenario);
		// print_nbody_scenario(&scenario);

		// Temporaries
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
    tla_Matrix *Phi     = tla_matrix_eye(&la, 6);
    tla_Matrix *Phi_dot = tla_matrix_eye(&la, 6);

		// Gravity tensor (3x3)
    tla_Matrix *G = tla_matrix_eye(&la, 3);
		size_t dim_y  = 6 * scenario.num_entities + 36; // Entities + STM
																													//
		// Integrator
    double *scratch = (double*) arena_alloc(&a, 5 * dim_y * sizeof(double));
    double *y       = (double*) arena_alloc(&a, dim_y * sizeof(double));

		memcpy(t_x,  scenario.x, scenario.num_entities * sizeof(double));
		memcpy(t_y,  scenario.y, scenario.num_entities * sizeof(double));
		memcpy(t_z,  scenario.z, scenario.num_entities * sizeof(double));
		memcpy(t_vx, scenario.vx,scenario.num_entities * sizeof(double));
		memcpy(t_vy, scenario.vy,scenario.num_entities * sizeof(double));
		memcpy(t_vz, scenario.vz,scenario.num_entities * sizeof(double));

		NBodyIntegrationContext ctx = {
			.scenario=&scenario, 
			.G=G,
			.Phi=Phi,
			.Phi_dot=Phi_dot,
			.t_x=t_x,
			.t_y=t_y,
			.t_z=t_z,
			.t_vx=t_vx,
			.t_vy=t_vy,    
			.t_vz=t_vz,
			.t_ax=t_ax,
			.t_ay=t_ay,    
			.t_az=t_az,
		};

		// Integrate leg by leg
		double dt        = 0.01;
		double current_t = 0;
		reset_stm(Phi, 6);
		n_body_integration_state_to_vec(
				y,
				Phi,
				scenario.num_entities,
				t_x,
				t_y,
				t_z,
				t_vx,
				t_vy,
				t_vz);

		for(size_t i = 0; i < scenario.num_nodes; i++){
			while(current_t + dt < scenario.t[i]){
				rk4_step(current_t, dt, y, n_body_ode_fn, dim_y, &ctx, scratch);
				current_t += dt;
			}
			double remaining_t = scenario.t[i] - current_t;
			if(remaining_t > 0){
				rk4_step(current_t, remaining_t, y, n_body_ode_fn, dim_y, &ctx, scratch);
				current_t += remaining_t;
			}

			// Apply impulsive maneuvers

			y[3] += scenario.dv_x[i];
			y[4] += scenario.dv_y[i];
			y[5] += scenario.dv_z[i];

			n_body_vec_to_integration_state(
					y,
					Phi,
					scenario.num_entities,
					t_x,
					t_y,
					t_z,
					t_vx,
					t_vy,
					t_vz);

			reset_stm(Phi, 6);

			n_body_integration_state_to_vec(
					y,
					Phi,
					scenario.num_entities,
					t_x,
					t_y,
					t_z,
					t_vx,
					t_vy,
					t_vz);


		}

		n_body_vec_to_integration_state(
				y,
				Phi,
				scenario.num_entities,
				t_x,
				t_y,
				t_z,
				t_vx,
				t_vy,
				t_vz);

    tla_print_matrix(G);

		memcpy(scenario.x,  t_x, scenario.num_entities * sizeof(double));
		memcpy(scenario.y,  t_y, scenario.num_entities * sizeof(double));
		memcpy(scenario.z,  t_z, scenario.num_entities * sizeof(double));
		memcpy(scenario.vx, t_vx, scenario.num_entities * sizeof(double));
		memcpy(scenario.vy, t_vy, scenario.num_entities * sizeof(double));
		memcpy(scenario.vz, t_vz, scenario.num_entities * sizeof(double));

		// print_nbody_scenario(&scenario);

		// Initialize simulation state

    arena_destroy(&a);
    tla_arena_destroy(&la);
    return 0;
}

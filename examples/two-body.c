#define TINY_KEPLER_IMPLEMENTATION
#include "tiny-kepler.h"

int main(int argc, char **argv) {

	// A satellite is initially at the following position, described
	// in a geocentric frame:

	double mu           = 398600;  // km3 / s2
	StateVector state_g = {
		.x  = -5102,     // km
		.y  = -8228,     // km
		.z  = -2105,     // km
		.vx = -4.348,    // km/s
		.vy = 3.478,     // km/s
		.vz = -2.846     // km/s
	};
	printf(
			"Initial state (geocentric): r=[%.2f, %.2f, %.2f], v=[%.2f, %.2f, %.2f]\n",
			state_g.x,
			state_g.y,
			state_g.z,
			state_g.vx,
			state_g.vy,
			state_g.vz
	);

	// Let's find its orbital elements

	OrbitalElements elements = state_to_orbital_elements(mu, &state_g);
	printf("Radial ascension (ra): %.2f\n", elements.ra);
	printf("Plane inclination (i): %.2f\n", elements.i);
	printf("Periapsis argument (w): %.2f\n", elements.w);
	printf("Eccentricity (e): %.2f\n", elements.e);
	printf("Specific angular momentum (h): %.2f\n", elements.h);
	printf("Real anomaly (theta): %.2f\n", elements.theta);

	// We'll now express our state in a perifocal frame
	
	geocentric_to_perifocal(&state_g, &elements);
	printf(
			"Initial state (perifocal): r=[%.2f, %.2f, %.2f], v=[%.2f, %.2f, %.2f]\n",
			state_g.x,
			state_g.y,
			state_g.z,
			state_g.vx,
			state_g.vy,
			state_g.vz
	);

	// We can now propagate the orbit in the perifocal frame

	double dt = 50 * 60; // 60 minutes

	StateVector2D state_peri = state3d_to_2d(&state_g);

	int flag  = 0;
	orbit_propagate_lagrange(&state_peri, mu, dt, &flag);
	if(flag != 0){
		printf("Error propagating orbit. Flag=%d\n", flag);
		exit(1);
	}

	// We can now express the new state back in the geocentric
	// reference frame

	StateVector new_state = state2d_to_3d(&state_peri);

	perifocal_to_geocentric(&new_state, &elements);

	printf(
			"Final state (geocentric): r=[%.2f, %.2f, %.2f], v=[%.2f, %.2f, %.2f]\n",
			new_state.x,
			new_state.y,
			new_state.z,
			new_state.vx,
			new_state.vy,
			new_state.vz
	);

  return 0;
}


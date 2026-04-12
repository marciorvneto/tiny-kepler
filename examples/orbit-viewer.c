#include <stdio.h>
#define TINY_KEPLER_IMPLEMENTATION
#include "tiny-kepler.h"
#include "raylib.h"

#define DEFAULT_SPEEDUP 1

typedef struct {
	Shader jacobi_shader;
} OrbitShaders;

OrbitShaders load_shaders(){
	OrbitShaders shaders;
	shaders.jacobi_shader = LoadShader(0, "./examples/shaders/jacobi.fs");
	return shaders;
}

void set_shader_uniforms(Camera2D *camera, SimpleSimulationResults *results, OrbitShaders *shaders, size_t frame){
	int screenWidth = GetScreenWidth();
	int screenHeight = GetScreenHeight();

	// Jacobi

	int muLoc  = GetShaderLocation(shaders->jacobi_shader, "mu");
	int csatLoc = GetShaderLocation(shaders->jacobi_shader, "C_sat");
	int resLoc  = GetShaderLocation(shaders->jacobi_shader, "resolution");
	int tgtLoc  = GetShaderLocation(shaders->jacobi_shader, "camTarget");
	int offLoc  = GetShaderLocation(shaders->jacobi_shader, "camOffset");
	int zoomLoc = GetShaderLocation(shaders->jacobi_shader, "camZoom");

	double C = cr3bp_jacobi_constant_normalized(
			results->mu,
			results->x[frame],
			results->y[frame],
			results->z[frame],
			results->vx[frame],
			results->vy[frame],
			results->vz[frame]);

	// Set static uniforms (resolution and mu don't change)
	float resolution[2] = { (float)screenWidth, (float)screenHeight };
	SetShaderValue(shaders->jacobi_shader, resLoc, resolution, SHADER_UNIFORM_VEC2);

	float mu_val = (float)results->mu;
	SetShaderValue(shaders->jacobi_shader, muLoc, &mu_val, SHADER_UNIFORM_FLOAT);

	// Update dynamic uniforms
	float C_sat_val = (float)C;
	SetShaderValue(shaders->jacobi_shader, csatLoc, &C_sat_val, SHADER_UNIFORM_FLOAT);

	float camTgt[2] = { camera->target.x, camera->target.y };
	SetShaderValue(shaders->jacobi_shader, tgtLoc, camTgt, SHADER_UNIFORM_VEC2);

	float camOff[2] = { camera->offset.x, camera->offset.y };
	SetShaderValue(shaders->jacobi_shader, offLoc, camOff, SHADER_UNIFORM_VEC2);

	float camZoom = camera->zoom;
	SetShaderValue(shaders->jacobi_shader, zoomLoc, &camZoom, SHADER_UNIFORM_FLOAT);

}

void get_initial_coords(
		SimpleSimulationResults *results,
		int screen_width,
		int screen_height,
		Camera2D *camera
		)
{
	if(results->mission_type == MISSION_TBP){
		camera->target = (Vector2){0, 0};
		camera->offset = (Vector2){screen_width/2.0, screen_height/2.0};
		return;
	}
	camera->target = (Vector2){0.5 - results->mu, 0};
	camera->offset = (Vector2){screen_width/2.0, screen_height/2.0};
}

void position_main_entities(
		SimpleSimulationResults *results,
		size_t frame,
		Vector2 *sc,
		Vector2 *earth,
		Vector2 *moon,
		float   *earth_radius,
		float   *moon_radius
)
{
	*earth_radius = 6378 / results->L;
	*moon_radius  = 1737.4 / results->L;
	if(results->mission_type == MISSION_CR3BP){
		sc->x    = results->x[frame];
		sc->y    = results->y[frame];

		earth->x = -results->mu;
		earth->y = 0;

		moon->x  = 1.0 - results->mu;
		moon->y  = 0;
		return;
	}

	sc->x    = results->x[frame];
	sc->y    = results->y[frame];

	earth->x = 0;
	earth->y = 0;
}

int main(int argc, char **argv)
{
		Arena a = arena_create(10 * 1024 * 1024);

		const char *path = argc > 1 ? argv[1] : "./results.out";
		SimpleSimulationResults results;
		read_simple_simulation_results(&a, path, &results);
		mission_t mission_type = results.mission_type;

		int speedup = results.display_options.speedup > 0 ? results.display_options.speedup : DEFAULT_SPEEDUP;

    const int screenWidth = 1024;
    const int screenHeight = 768;
		Color bg_color = {18, 18, 18, 255};

    InitWindow(screenWidth, screenHeight, "Tiny Kepler Orbit Viewer");
    SetTargetFPS(60);

		OrbitShaders shaders = load_shaders();

		Camera2D camera = {0};
		get_initial_coords(&results, screenWidth, screenHeight, &camera);
		camera.zoom = (screenHeight * 0.4f);

		// Preallocating entities
		Vector2 sc;
		Vector2 earth;
		Vector2 moon;
		float earth_radius, moon_radius;

		size_t frame = 0;
		char buf[256];
		while (!WindowShouldClose())
		{
			if(IsKeyDown(KEY_EQUAL))  camera.zoom *= 1.02;
			if(IsKeyDown(KEY_MINUS))  camera.zoom *= 0.98;
			float pan_speed = 10.0 / camera.zoom;
			if(IsKeyDown(KEY_UP))     camera.target.y -= pan_speed;
			if(IsKeyDown(KEY_DOWN))   camera.target.y += pan_speed;
			if(IsKeyDown(KEY_LEFT))   camera.target.x -= pan_speed;
			if(IsKeyDown(KEY_RIGHT))  camera.target.x += pan_speed;
			int wheel = GetMouseWheelMove();
			if(wheel != 0){
				camera.zoom *= powf(1.1, wheel);
			}

			set_shader_uniforms(&camera, &results, &shaders, frame);

			BeginDrawing();
			ClearBackground(bg_color);

			if(mission_type == MISSION_CR3BP && results.display_options.show_jacobi){
				BeginShaderMode(shaders.jacobi_shader);
					DrawRectangle(0, 0, screenWidth, screenHeight, BLANK);
				EndShaderMode();
			}

			BeginMode2D(camera);


			// Trail
			if(frame > 0){
				for(size_t i = 0; i < frame - 1; i++){
					// unsigned char alpha = (unsigned char)((float)i / frame * 255);
					unsigned char alpha = 255;
					DrawLineV(
							(Vector2){results.x[i], results.y[i]},
							(Vector2){results.x[i+1], results.y[i+1]},
							(Color){0, 228, 48, alpha} // GREEN with alpha
							);
				}
			}

			position_main_entities(&results, frame, &sc, &earth, &moon, &earth_radius, &moon_radius);

			// Spacecraft
			DrawCircleV(sc, 4.0/camera.zoom, RED);

			float earth_draw_r = earth_radius;
			if (earth_draw_r * camera.zoom < 2.0f) earth_draw_r = 2.0f / camera.zoom;
			DrawCircleV(earth, earth_draw_r, BLUE);
			DrawCircleLinesV(earth, earth_draw_r, SKYBLUE);
			if(mission_type == MISSION_CR3BP){
				float moon_draw_r = moon_radius;
				if (moon_draw_r * camera.zoom < 1.5f) moon_draw_r = 1.5f / camera.zoom;
				DrawCircleV(moon, moon_draw_r, GRAY);
			}
			EndMode2D();

			double V_norm = results.L / results.T;
			double v = sqrt(SQR(results.vx[frame]) + SQR(results.vy[frame]));
			double t_real = results.t[frame] * results.T;

			sprintf(buf, "t = %.0fs", t_real);
			DrawText(buf, 10, 10, 24, LIGHTGRAY);
			sprintf(buf, "V = %.2f km/s", v * V_norm);
			DrawText(buf, 10, 40, 24, LIGHTGRAY);


			EndDrawing();

				frame = (frame + speedup) % results.num_results;
    }

    CloseWindow();
		arena_destroy(&a);
    return 0;
}


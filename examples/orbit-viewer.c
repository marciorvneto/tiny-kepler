#include <stdio.h>
#define TINY_KEPLER_IMPLEMENTATION
#include "tiny-kepler.h"
#include "raylib.h"

#define MULT 800
#define SPEEDUP 1

typedef struct {
	Shader jacobi_shader;
} OrbitShaders;

OrbitShaders load_shaders(){
	OrbitShaders shaders;
	shaders.jacobi_shader = LoadShader(0, "./examples/shaders/jacobi.fs");
	return shaders;
}

void set_shader_uniforms(Camera2D *camera, CR3BPResults *results, OrbitShaders *shaders, size_t frame){
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

int main(int argc, char **argv)
{
		Arena a = arena_create(10 * 1024 * 1024);

		const char *path = argc > 1 ? argv[1] : "./results.out";
		CR3BPResults results;
		read_cr3bp_result(&a, path, &results);

    const int screenWidth = 1024;
    const int screenHeight = 768;
		Color bg_color = {18, 18, 18, 255};

    InitWindow(screenWidth, screenHeight, "Tiny Kepler Orbit Viewer");
    SetTargetFPS(60);

		OrbitShaders shaders = load_shaders();

		Camera2D camera = {0};
		camera.target = (Vector2){0.5 - results.mu, 0};
		camera.offset = (Vector2){screenWidth/2.0, screenHeight/2.0};
		camera.zoom = MULT;

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

			if(results.display_options.show_jacobi){
				BeginShaderMode(shaders.jacobi_shader);
					DrawRectangle(0, 0, screenWidth, screenHeight, BLANK);
				EndShaderMode();
			}

			BeginMode2D(camera);


			// Trail
			if(frame > 0){
				for(size_t i = 0; i < frame - 1; i++){
					DrawLineV(
							(Vector2){results.x[i], results.y[i]},
							(Vector2){results.x[i+1], results.y[i+1]},
							GREEN
							);
				}
			}
			// Spacecraft
			DrawCircleV((Vector2){results.x[frame], results.y[frame]}, 3.0/camera.zoom, RED);
			DrawCircleV((Vector2){-results.mu, 0}, 3.0/camera.zoom, BLUE);
			DrawCircleV((Vector2){1.0 - results.mu, 0}, 2.0/camera.zoom, GRAY);
			EndMode2D();

			double V_norm = results.L / results.T;
			double v = sqrt(SQR(results.vx[frame]) + SQR(results.vy[frame]));
			double t_real = results.t[frame] * results.T;

			sprintf(buf, "t = %.0fs", t_real);
			DrawText(buf, 10, 10, 24, LIGHTGRAY);
			sprintf(buf, "V = %.2f km/s", v * V_norm);
			DrawText(buf, 10, 40, 24, LIGHTGRAY);


			EndDrawing();

				frame = (frame + SPEEDUP) % results.num_results;
    }

    CloseWindow();
		arena_destroy(&a);
    return 0;
}


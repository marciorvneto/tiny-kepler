#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct {
    float x, y;
} EntityPos;

typedef struct {
    float t;
    EntityPos *entities;
} Frame;

int main(int argc, char **argv) {
    const char *path = (argc > 1) ? argv[1] : "n-body-results.out";
    FILE *f = fopen(path, "rb");
    if (!f) {
        printf("Could not open %s\n", path);
        return 1;
    }

    uint32_t num_entities;
    fread(&num_entities, sizeof(uint32_t), 1, f);

    size_t capacity = 10000;
    Frame *frames = malloc(sizeof(Frame) * capacity);
    int frame_count = 0;

    // Load data from binary dump
    while (!feof(f)) {
        if (frame_count >= capacity) {
            capacity *= 2;
            frames = realloc(frames, sizeof(Frame) * capacity);
        }
        if (fread(&frames[frame_count].t, sizeof(float), 1, f) != 1) break;
        frames[frame_count].entities = malloc(sizeof(EntityPos) * num_entities);
        fread(frames[frame_count].entities, sizeof(EntityPos), num_entities, f);
        frame_count++;
    }
    fclose(f);

    InitWindow(1280, 720, "N-Body Mission Visualizer - Voima");
    SetTargetFPS(60);

    // Initial Camera Setup
    Camera2D camera = { 0 };
    camera.zoom = 300.0f;
    camera.offset = (Vector2){ 640, 360 };
    
    int current_frame = 0;
    int focus_id = 1; // Default to Sun
    int playback_speed = 5;
    bool paused = false;

    while (!WindowShouldClose()) {

        // Toggle Pause
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;

        // Cycle Focus
        if (IsKeyPressed(KEY_TAB)) focus_id = (focus_id + 1) % num_entities;

        // Playback Speed
        if (IsKeyPressed(KEY_RIGHT)) playback_speed += 5;
        if (IsKeyPressed(KEY_LEFT)) playback_speed = (playback_speed > 5) ? playback_speed - 5 : 1;

        // Panning (Right Mouse Button)
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 delta = GetMouseDelta();
            delta = Vector2Scale(delta, -1.0f / camera.zoom);
            camera.target = Vector2Add(camera.target, delta);
            focus_id = -1; // Break focus follow on manual pan
        }

        // Zooming (Mouse Wheel)
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
            camera.offset = GetMousePosition();
            camera.target = mouseWorldPos;

            float scaleFactor = 1.0f + wheel * 0.15f;
            camera.zoom *= scaleFactor;
            
            if (camera.zoom < 0.1f) camera.zoom = 0.1f;
            if (camera.zoom > 1000000.0f) camera.zoom = 1000000.0f;
        }

        // Auto-follow focused entity
        if (focus_id != -1) {
            camera.target = (Vector2){ frames[current_frame].entities[focus_id].x, 
                                       frames[current_frame].entities[focus_id].y };
        }

				// TODO: Make this general! Should read from the input file
        BeginDrawing();
            ClearBackground((Color){10, 10, 15, 255});
            
            BeginMode2D(camera);
                for (int e = 0; e < (int)num_entities; e++) {
                    Color color = GREEN;
                    if (e == 0) color = RAYWHITE; // Craft
                    if (e == 1) color = YELLOW;   // Sun
                    if (e == 2) color = BLUE;     // Earth
                    if (e == 3) color = GRAY;     // Moon
                    if (e == 4) color = RED;      // Mars

                    // Draw Trails
                    for (int i = 0; i < current_frame - 5; i += 5) {
                        Vector2 p1 = { frames[i].entities[e].x, frames[i].entities[e].y };
                        Vector2 p2 = { frames[i+5].entities[e].x, frames[i+5].entities[e].y };
                        DrawLineV(p1, p2, Fade(color, 0.2f));
                    }

                    // Draw Body
                    Vector2 pos = { frames[current_frame].entities[e].x, 
                                    frames[current_frame].entities[e].y };
                                    
                    float baseRadius = (e == 1) ? 15.0f : 5.0f;
                    float drawRadius = baseRadius / camera.zoom;
                    if (drawRadius < 2.0f / camera.zoom) drawRadius = 2.0f / camera.zoom; 
                    
                    DrawCircleV(pos, drawRadius, color);
                }
            EndMode2D();

            Vector2 craftWorldPos = { frames[current_frame].entities[0].x, 
                                      frames[current_frame].entities[0].y };
            Vector2 craftScreenPos = GetWorldToScreen2D(craftWorldPos, camera);

            // Draw a pulsing targeting reticle around the craft
            float pulse = 15.0f + sinf(GetTime() * 5.0f) * 3.0f;
            DrawCircleLines((int)craftScreenPos.x, (int)craftScreenPos.y, pulse, Fade(GREEN, 0.8f));
            DrawLine((int)craftScreenPos.x - 25, (int)craftScreenPos.y, (int)craftScreenPos.x + 25, (int)craftScreenPos.y, Fade(GREEN, 0.4f));
            DrawLine((int)craftScreenPos.x, (int)craftScreenPos.y - 25, (int)craftScreenPos.x, (int)craftScreenPos.y + 25, Fade(GREEN, 0.4f));
            
            // Craft Label
            DrawText("VOIMA-1", (int)craftScreenPos.x + 20, (int)craftScreenPos.y - 20, 20, GREEN);

						// UI
            DrawRectangle(10, 10, 320, 150, Fade(BLACK, 0.7f));
            DrawText(TextFormat("TIME: %.2f days", frames[current_frame].t * 58.1), 20, 20, 20, RAYWHITE);
            DrawText(TextFormat("SPEED: %dx", playback_speed), 20, 45, 20, RAYWHITE);
            DrawText(TextFormat("ZOOM: %.1fx", camera.zoom), 20, 70, 20, RAYWHITE);
            
            const char* focusName = (focus_id == -1) ? "Manual" : TextFormat("Entity %d", focus_id);
            DrawText(TextFormat("FOCUS: %s", focusName), 20, 95, 20, RAYWHITE);
            DrawText("R-CLICK: Pan | WHEEL: Zoom | TAB: Focus", 20, 125, 15, GRAY);

        EndDrawing();

        if (!paused) {
            current_frame += playback_speed;
            if (current_frame >= frame_count) current_frame = 0;
        }
    }

    // Cleanup
    for(int i=0; i<frame_count; i++) free(frames[i].entities);
    free(frames);
    CloseWindow();
    return 0;
}

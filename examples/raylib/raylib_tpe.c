#include "raylib.h"
#include "tinyphysicsengine.h"  // Include the header-only library

#include <stdio.h>   // For printf (debugging)
#include <math.h>    // For cosf, sinf
#include <string.h>  // For memset

// Constants
#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 600
#define PHYSICS_STEPS_PER_FRAME 4  // Multiple fixed steps for stability (TPE internal dt ~1/60s)
#define GRAVITY -9.81f  // m/s²
#define MAX_BODIES 16   // Adjust based on needs (TPE default allows up to 256, but keep small)
#define MAX_SHAPES 32   // Global pool size
#define MAX_JOINTS 8    // Usually 0 for simple sim

// Forward declarations
void UpdatePhysics(TPE_World* world);

TPE_Vec3 environmentDistance(TPE_Vec3 point, TPE_Unit maxDistance){
  return TPE_envGround(point,0); // just an infinite flat plane
}

// Main entry point
int main(void) {
    // Initialization
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Raylib + TinyPhysicsEngine 3D Physics Demo");
    SetTargetFPS(60);

    // Setup 3D camera (orbiting)
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    TPE_World world;
    TPE_Body body;
    TPE_Joint joint;
    int frame = 0;

    // Initialize world with pre-allocated arrays
    joint = TPE_joint(TPE_vec3(0,TPE_F * 8,0),TPE_F);
    TPE_bodyInit(&body, &joint, 1, 0, 0, 2 * TPE_F);

    TPE_worldInit(&world, &body, 1, environmentDistance);
    Vector3 cubePos = {0,0,0};
    Vector3 cubeSize = {1,1,1};

    // Game loop
    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();  // Unused for TPE, but for camera

        // Update camera (simple mouse orbit)
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            float mouseX = (float)GetMouseX();
            float mouseY = (float)GetMouseY();
            camera.position.x = cosf(mouseX * 0.01f) * 15.0f;
            camera.position.z = sinf(mouseX * 0.01f) * 15.0f;
            camera.position.y = 5.0f + sinf(mouseY * 0.01f) * 5.0f;
        }
        UpdateCamera(&camera, CAMERA_CUSTOM);  // Manual control

        // Step physics (multiple calls to TPE_worldStep for substeps)
        // for (int i = 0; i < PHYSICS_STEPS_PER_FRAME; ++i) {
            UpdatePhysics(&world);
        // }

        if(TPE_bodyIsActive(&body)){
            if (frame % 6 == 0) // print once in 6 frames
            {
            TPE_Unit height = TPE_bodyGetCenterOfMass(&body).y;
            // cubePos.x=0.0f;
            // cubePos.y= (float)height;
            // cubePos.z=0.0f;

            TPE_Vec3 com = TPE_bodyGetCenterOfMass(&world.bodies[0]);
            cubePos.x = (float)com.x / TPE_F;
            cubePos.y = (float)com.y / TPE_F;
            cubePos.z = (float)com.z / TPE_F;

            for (int i = 0; i < (height * 4) / TPE_F; ++i)
                putchar(' ');
                puts("*");
            }

            TPE_bodyApplyGravity(&body,TPE_F / 100);
            // TPE_worldStep(&world);
            frame++;
        }

        if(IsKeyPressed(KEY_R)){
            printf("reset \n");
            TPE_bodyActivate(&body);
            TPE_bodyMoveTo(&body, TPE_vec3(0,20000,0));
            // TPE_bodyMoveBy(&body, TPE_vec3(0,2000,0));

        }

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                DrawGrid(10,10);
                DrawCubeWires(cubePos, cubeSize.x, cubeSize.y, cubeSize.z, RED);
                DrawCubeWires((Vector3){0,0,0}, cubeSize.x, cubeSize.y, cubeSize.z, BLACK);
                // DrawScene(&camera, cubeBody, (Vector3){1.0f, 1.0f, 1.0f});  // Full size = 2 * halfExtents
            EndMode3D();
            DrawFPS(10, 10);

            DrawText("Reset Cube = R Key", 10, 40, 20, DARKGRAY);
            const char *cubePosText = 0;
            cubePosText = TextFormat("pos x:%0.0f y:%0.0f z:%0.0f", cubePos.x , cubePos.y, cubePos.z);
            DrawText(cubePosText, 10, 60, 20, DARKGRAY);

        EndDrawing();
    }

    // Cleanup
    CloseWindow();
    return 0;
}

// Update physics (apply gravity, step simulation)
void UpdatePhysics(TPE_World* world) {
    // Apply gravity to dynamic bodies (force = mass * g)
    for (uint16_t i = 0; i < world->bodyCount; ++i) {
        TPE_Body* body = &world->bodies[i];
        // if (body->mass > 0.0f) {  // Dynamic only
        //     body->force.y += body->mass * GRAVITY;  // Accumulate on force member
        //     // Note: TPE integrates force -> velocity -> position in step (Euler or similar)
        // }

    }

    // Step the world (fixed timestep, handles collision/response)
    TPE_worldStep(world);
}
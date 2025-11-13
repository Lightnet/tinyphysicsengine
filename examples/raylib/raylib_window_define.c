#include "raylib.h"
#include <stdio.h>

#define TPE_FLOAT_MODE // use float

#ifdef TPE_FLOAT_MODE
    #define TPE_USE_FLOAT 1
#else
    #define TPE_USE_FLOAT 0
#endif

int main(void)
{
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");
    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second

#if TPE_USE_FLOAT 
    printf("FLOAT TEST\n");
#else
    printf("INT TEST\n");
#endif

    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        BeginDrawing();
            ClearBackground(RAYWHITE);
            DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);
        EndDrawing();
    }
    CloseWindow();        // Close window and OpenGL context
    return 0;
}
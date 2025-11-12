#include <SDL2/SDL.h>
#include <stdbool.h> // for bool type

int main(int argc, char* args[]) {
    // 1. Initialize SDL2 subsystems (video, events, etc.)
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        // Report error if initialization fails
        SDL_Log("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
        return 1;
    }

    // 2. Create a window and renderer
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    int window_width = 640;
    int window_height = 480;

    // SDL_CreateWindowAndRenderer is a convenience function that creates both
    if (SDL_CreateWindowAndRenderer(window_width, window_height, SDL_WINDOW_SHOWN, &window, &renderer) < 0) {
        SDL_Log("Window and renderer could not be created! SDL Error: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    // Set draw color to black and clear the renderer
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    // 3. Main event loop to keep the window open
    bool quit = false;
    SDL_Event e;

    while (!quit) {
        // Handle events on queue
        while (SDL_PollEvent(&e) != 0) {
            // User requests quit
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }
        // You would place rendering/drawing code here in a more complex application
    }

    // 4. Clean up resources and quit SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

#include <SDL.h>
#include <iostream>
#include <chrono>

#include "Drone.h"
#include "Physics.h"

int main(int argc, char* argv[])
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL Init Failed\n";
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Drone Simulator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        800,
        600,
        SDL_WINDOW_SHOWN
    );

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (!window || !renderer)
    {
        std::cerr << "Window or Renderer creation failed\n";
        return -1;
    }

    // Create physics + drone state
    PhysicsEngine physics(1.0);
    State state{};
    Inputs inputs{};

    bool running = true;
    SDL_Event event;

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (running)
    {
        // Calculate delta time
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> delta = currentTime - lastTime;
        lastTime = currentTime;
        double dt = delta.count();

        // Handle input
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
                running = false;
        }

        const Uint8* keyboard = SDL_GetKeyboardState(NULL);

        // Reset inputs
        inputs.thrust = 9.81; // hover baseline
        inputs.roll_torque = 0.0;
        inputs.pitch_torque = 0.0;
        inputs.yaw_torque = 0.0;

        if (keyboard[SDL_SCANCODE_W]) inputs.thrust += 5.0;
        if (keyboard[SDL_SCANCODE_S]) inputs.thrust -= 5.0;
        if (keyboard[SDL_SCANCODE_A]) inputs.roll_torque = -0.5;
        if (keyboard[SDL_SCANCODE_D]) inputs.roll_torque = 13.0;

        // Step physics
        physics.step(state, inputs, dt);

        // Render
        SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255);
        SDL_RenderClear(renderer);

        // Convert world position to screen
        int screenX = 400;
        int screenY = 500 - static_cast<int>(state.z * 50.0);

        SDL_Rect droneRect = { screenX - 20, screenY - 10, 40, 20 };

        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        SDL_RenderFillRect(renderer, &droneRect);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
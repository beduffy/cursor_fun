/* Minimal SDL2 window (optional) */
#include <SDL2/SDL.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  SDL_Window *win;
  SDL_Renderer *ren;
  SDL_Event e;
  int running = 1;
  if (SDL_Init(SDL_INIT_VIDEO) != 0) { printf("SDL init failed\n"); return 1; }
  win = SDL_CreateWindow("C89 SDL2 Demo", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, 0);
  ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
  while (running) {
    while (SDL_PollEvent(&e)) { if (e.type == SDL_QUIT) running = 0; }
    SDL_SetRenderDrawColor(ren, 30, 80, 150, 255);
    SDL_RenderClear(ren);
    SDL_SetRenderDrawColor(ren, 255, 255, 0, 255);
    SDL_RenderDrawLine(ren, 50, 50, 590, 430);
    SDL_RenderPresent(ren);
  }
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(win);
  SDL_Quit();
  return 0;
}




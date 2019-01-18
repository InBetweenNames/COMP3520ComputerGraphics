// main.cpp : This file contains the 'main' function. Program execution begins and ends there.

//Standard library includes
#include <cstdio> //when you want to include a C standard library header, like stdio.h, use cstdio instead

//SDL2 includes
#include <SDL.h>
#include <SDL_ttf.h>

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

//The example code given is C code, but feel free to use C++ at any time in your work.

int main(int argc, char* args[]) {

	//VERY IMPORTANT: Ensure SDL2 is initialized
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not initialize sdl2: %s\n", SDL_GetError());
		return 1;
	}
	
	//VERY IMPORTANT: if using text in your program, ensure SDL2_ttf library is initialized
	if (TTF_Init() < 0)
	{
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not initialize SDL2_ttf: %s\n", TTF_GetError());
	}

	//This creates the actual window in which graphics are displayed
	SDL_Window* window = SDL_CreateWindow(
		"hello_sdl2",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		SCREEN_WIDTH, SCREEN_HEIGHT,
		SDL_WINDOW_SHOWN
	);

	//Check for errors: window will be NULL if there was a problem
	if (window == NULL) {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create window: %s\n", SDL_GetError());
		return 1;
	}

	//A window by itself can't do many useful things.  We need a renderer so that we can paint in this window.
	//Think of a renderer like a paint brush.
	//First, let's instantiate a renderer in the window using the system's preferred graphics API.
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0); //the "0" lets the SDL2 choose what's best.

	if (renderer == NULL) {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create renderer: %s\n", SDL_GetError());
		return 1;
	}

	//OK, now we have the window and the rendering context.
	//Let's create our virtual "canvas" that we'll use to paint on.
	//This will live entirely on the CPU, and is stored in an SDL_Surface.
	//SDL_Surfaces are always CPU-only objects.
	//Note that we are requesting an RGBA8 surface.
	SDL_Surface* canvas = SDL_CreateRGBSurfaceWithFormat(0, SCREEN_WIDTH, SCREEN_HEIGHT, 32, SDL_PIXELFORMAT_RGBA8888);
	if (canvas == NULL) {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create surface: %s\n", SDL_GetError());
		return 1;
	}

	//Unfortunately, we can't output CPU surfaces directly to the window with the renderer.  We have to copy
	//it over first.  Create a texture on the GPU that will receive our rendered images.  Consider it to be the GPU
	//side of our canvas.
	//The pixel format should be in agreement with the surface given.
	SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING,
		SCREEN_WIDTH, SCREEN_HEIGHT);
	//SDL_TEXTUREACCESS_STREAMING allows the texture to be streamed from the CPU.

	if (texture == NULL)
	{
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create texture: %s\n", SDL_GetError());
		return 1;
	}

	// Just in case you need text:
	// load iosevka-regular.ttf at a large size into font
	TTF_Font* font = TTF_OpenFont("iosevka-regular.ttf", 64);
	if (!font) {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "TTF_OpenFont: %s\n", TTF_GetError());
		return 1;
	}

	//OK, now we're all set to start rendering.  Let's paint a white background.
	//FillRect essentially fills a rectangle of pixels with a solid colour for us.
	SDL_FillRect(canvas, NULL, SDL_MapRGB(canvas->format, 0xFF, 0xFF, 0xFF));

	//Now, lets draw some pixel-sized points.

	//uint32_t* pixels = (uint32_t*) canvas->pixels; //Recall that an RGBA8 pixel is 32 bits wide
	uint32_t (*pixels)[SCREEN_WIDTH] = (uint32_t(*)[SCREEN_WIDTH]) canvas->pixels;
	
	//Draw a pixel at 100,100:
	//*(pixels + SCREEN_WIDTH * 150 + 100) = 0x000000FF;
	pixels[150][100] = 0x000000FF;

	//Draw a red line at a -45 degrees angle:
	for (size_t i = 0; i < SCREEN_HEIGHT; i++)
	{
		//*(pixels + SCREEN_WIDTH * i + i) = 0xFF0000FF;
		pixels[i][i] = 0xFF0000FF;
	}

	//Draw a green line at a 45 degrees angle:
	for (size_t i = 0; i < SCREEN_HEIGHT; i++)
	{
		//*(pixels + SCREEN_WIDTH * (SCREEN_HEIGHT - i) + i) = 0x00FF00FF;
		pixels[SCREEN_HEIGHT - i][i] = 0x00FF00FF;
	}


	//Exercise: can you do both the above loops in one loop?
	
	//Now that we have painted a complete picture, we're ready to display the results to the user.
	//Copy our software-rendered canvas over to the GPU using the texture we created.
	SDL_UpdateTexture(texture, NULL, canvas->pixels, canvas->pitch);
	//Now draw this directly to the window
	SDL_RenderCopy(renderer, texture, NULL, NULL);
	//And ask the windowing system to redraw the window
	SDL_RenderPresent(renderer);

	//Wait for input on the command line
	puts("Press any key in the terminal to exit");
	int const input = getchar();

	//VERY IMPORTANT: free your resources when you are done with them.
	SDL_DestroyTexture(texture);
	SDL_FreeSurface(canvas);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window); 

	//SDL_Quit() handles program exit, so no "return 0" statement is necessary.
	SDL_Quit();

	return 0; //Include a return statement anyway to make the compiler happy.

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

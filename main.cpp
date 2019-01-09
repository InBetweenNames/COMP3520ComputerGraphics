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
		fprintf(stderr, "could not initialize sdl2: %s\n", SDL_GetError());
		return 1;
	}
	
	//VERY IMPORTANT: if using text in your program, ensure SDL2_ttf library is initialized
	if (TTF_Init() < 0)
	{
		fprintf(stderr, "could not initialize SDL2_ttf: %s\n", TTF_GetError());
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
		fprintf(stderr, "could not create window: %s\n", SDL_GetError());
		return 1;
	}

	//A window by itself can't do many useful things.  We need a surface to display graphics in a window.
	//Get a handle to the implicit surface that the window we just created has, and use that to draw some graphics.
	SDL_Surface* screenSurface = SDL_GetWindowSurface(window);

	//OK, so now that we have the handle, let's paint the background a solid colour
	SDL_FillRect(screenSurface, NULL, SDL_MapRGB(screenSurface->format, 0xFF, 0xFF, 0xFF));
	
	//A "Hello World" wouldn't be much without a nice greeting, would it?
	//Let's greet the user using SDL2_ttf to render a message.


	// load iosevka-regular.ttf at a large size into font
	TTF_Font* font = TTF_OpenFont("iosevka-regular.ttf", 64);
	if (!font) {
		fprintf(stderr, "TTF_OpenFont: %s\n", TTF_GetError());
		return 1;
	}

	// Render some text in solid black to a new surface
	// then blit to the upper left of the screen
	// then free the text surface
	//SDL_Surface *screen;
	SDL_Color fg = { 0,0,0 };
	//SDL_Color bg = { 0xFF,0xFF,0xFF };
	SDL_Surface *text_surface;
	if (!(text_surface = TTF_RenderText_Blended(font, "Hello World!", fg))) {
		//handle error here, perhaps print TTF_GetError at least
	}
	else {
		SDL_BlitSurface(text_surface, NULL, screenSurface, NULL);
		//perhaps we can reuse it, but I assume not for simplicity.
		SDL_FreeSurface(text_surface);
	}

	//Now that we have painted a complete picture, we're ready to display the results to the user.
	//Inform the windowing system that we'd like to be redrawn on the user's screen.
	SDL_UpdateWindowSurface(window);

	//Hold this image for about 2 seconds, then exit the program.
	SDL_Delay(2000);
	SDL_DestroyWindow(window); //VERY IMPORTANT: free your resources when you are done with them.

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

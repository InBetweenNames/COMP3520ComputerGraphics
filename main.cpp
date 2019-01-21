#include <SDL.h>
#include <SDL_ttf.h>

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>

inline void Draw(SDL_Surface *);
inline void MovePlayer(int);
inline int ProcessEvent();
inline void DrawSpan(
		SDL_Surface *surface,
		SDL_Surface *tex,
		int x1,
		int x2,
		int y);



//Render properties
struct render{
	uint16_t xres; //X resolution
	uint16_t yres; //Y resolution
	uint16_t FPS; //Frame rate
}render = {640,480,10}; //Defaults: 320x240 at 35fps (DOOM)


struct textures{
	SDL_Surface *floor;
	SDL_Surface *sky;
}tex = {};

//Key states
struct movement{
	char xdiff; //Whether A(-1) or D(1) is down
	char ydiff; //Whether W(1) or S(-1) is down
	char hdiff; //Whether T(1) or G(-1) is down
	char cdiff; //Whether Y(-1) or H(1) is down
}move = {};

//Player location, direction, and speed
struct direction{
	//Direction vector
	float px;
	float py;

	//Normal vector
	float nx;
	float ny;

	//Player angle from X axis
	float theta;

	//Player position
	float xpos;
	float ypos;

	//Camera properties
	float hpos; //Height
	float cpos; //Distance from lens
	
	//Player speed
	float speed; //Units per millisecond
}dir = {
	1.0,0.0,
	0.0,1.0,
	0.0f,
	0.0f,0.0f,
	80.0f,400.0f, 
	192.0f/1000.0f};

int main(int argc, char *argv[]){

    //VERY IMPORTANT: Ensure SDL2 is initialized
	if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER) < 0) {
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
		"Floor_SDL2",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		render.xres, render.yres,
		SDL_WINDOW_SHOWN
    );
	
	if (window == NULL){
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not create window!\n");
		return 1;
	}
	
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0); //the "0" lets the SDL2 choose what's best.

	if (renderer == NULL) {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create renderer: %s\n", SDL_GetError());
		return 1;
    }
    
    SDL_Surface* display = SDL_CreateRGBSurfaceWithFormat(0, render.xres, render.yres, 32, SDL_PIXELFORMAT_RGBA8888);
	if (display == NULL) {
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create surface: %s\n", SDL_GetError());
		return 1;
    }
    
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING,
    render.xres, render.yres);
    
    if (texture == NULL)
	{
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create texture: %s\n", SDL_GetError());
		return 1;
    }
    
    // Just in case you need text:
    // load iosevka-regular.ttf at a large size into font
    TTF_Font* font = TTF_OpenFont("iosevka-regular.ttf", 16);
    if (!font) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "TTF_OpenFont: %s\n", TTF_GetError());
        return 1;
    }

	SDL_Surface *tmp = SDL_LoadBMP("floor.bmp");

	if (tmp == NULL){
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not load floor.bmp!\n");
		return 1;
	}

	tex.floor = SDL_ConvertSurface(tmp, display->format, 0);

	SDL_FreeSurface(tmp);

	tmp = SDL_LoadBMP("sky.bmp");

	if (tmp == NULL){
		SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not load sky.bmp!\n");
		return 1;
	}

	tex.sky = SDL_ConvertSurface(tmp, display->format, 0);

	SDL_FreeSurface(tmp);

	uint32_t starttime;
	uint32_t endtime, timediff;

	uint32_t gameRunning = 1;

	float MPF = 1000.0f/render.FPS; //milliseconds per frame

	//Only render a frame if at least the MPF has been passed in time
	//If the  time has not been passed, pop and process one event from
	//the event queue.  So, for fast computers, many events will be processed
	//in a single frame.  But for slow computers, only one event will be
	//popped and processed in a single frame.  Beneficial because on a slow
	//computer, if all events were popped at once, if the W key were tapped
	//for instance, it would add to the ydiff variable and subtract from it
	//before MovePlayer would have a chance to do anything.  In this case,
	//the player might move too far, but at least the player moved.

	timediff = MPF + 1;
	while (gameRunning){
		if (timediff >= MPF){
			starttime = SDL_GetTicks();
			Draw(display);
            SDL_UpdateTexture(texture, NULL, display->pixels, display->pitch);
            //Now draw this directly to the window
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            //And ask the windowing system to redraw the window
            SDL_RenderPresent(renderer);
			MovePlayer(timediff);
		}
		gameRunning = ProcessEvent();
		if (timediff < MPF && gameRunning == 2){
			SDL_Delay(1);
		}
		endtime = SDL_GetTicks();
		timediff = endtime - starttime;
		//printf("%d\n", timediff);
	}
	
	SDL_FreeSurface(tex.floor);
    SDL_FreeSurface(tex.sky);
	SDL_DestroyTexture(texture);
	SDL_FreeSurface(display);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);

	return 0;
}

inline int ProcessEvent(){

	char e = 0;

	static SDL_Event event;

	if (SDL_PollEvent(&event)){
		switch (event.type){
			default:
				break;

			case SDL_QUIT:
				return 0; //0 -- signal terminate program

			case SDL_KEYUP:
			case SDL_KEYDOWN:

				//Because I'm lazy: e is positive or negative depending
				e = SDL_KEYDOWN == event.type ? 1 : -1;

				switch (event.key.keysym.sym){
					case SDLK_w: //forward
						move.ydiff += e;
					break;
					case SDLK_s: //backward
						move.ydiff -= e;
					break;
					case SDLK_a: //left
						move.xdiff -= e;
					break;
					case SDLK_d: //right
						move.xdiff += e;
					break;
					case SDLK_t: //height up
						move.hdiff += e;
					break;
					case SDLK_g: //height down
						move.hdiff -= e;
					break;
					case SDLK_y: //viewer-to-camera distance decrease (zoom out)
						move.cdiff -= e; //slower
					break;
					case SDLK_h: //viewer-to-camera distance increase (zoom in)
						move.cdiff += e; //slower
                    default:
                    break;
				}
				break;
			case SDL_MOUSEMOTION:

				if (SDL_GetRelativeMouseMode() == SDL_TRUE){
					dir.theta += event.motion.xrel;
					//Ugly, but it works
					dir.theta /= 360;
					dir.theta = dir.theta - floor(dir.theta);
					dir.theta *= 360;
				}

				break;

			case SDL_MOUSEBUTTONUP:
				SDL_bool e = SDL_ShowCursor(SDL_QUERY) == SDL_ENABLE ? SDL_TRUE : SDL_FALSE;
                SDL_SetRelativeMouseMode(e);
				break;
		}
	}else return 2; //2 - signal nothing processed
	return 1; //1 - signal something processed

}

inline void MovePlayer(int timediff){

	dir.px = cos((dir.theta/180)*M_PI);
	dir.py = sin((dir.theta/180)*M_PI);

	dir.nx = -dir.py;
	dir.ny = dir.px;

	//Travel along normal
	dir.xpos += move.xdiff*dir.speed*timediff*dir.nx;
	dir.ypos += move.xdiff*dir.speed*timediff*dir.ny;

	//Travel along direction vector
	dir.xpos += move.ydiff*dir.speed*timediff*dir.px;
	dir.ypos += move.ydiff*dir.speed*timediff*dir.py;

	dir.hpos += move.hdiff*dir.speed*timediff;
	dir.cpos += move.cdiff*dir.speed*timediff;


}

inline void Draw(SDL_Surface *display){

	//Render sky
	SDL_Rect rect = {0,0, display->w, (display->h/2 + 1)};
	SDL_FillRect(display, &rect, 0xAAAAAAAA);
    
    //TODO:
	//SDL_BlitSurface(tex.sky, NULL, display, &rect);

	//Render floor
	for (int i = display->h/2 + 1; i < display->h; i++){
		DrawSpan(display, tex.floor, 0, render.xres - 1, i);
	}

}

//Tex must be 64x64.
//0 <= x1 < x <= surface->w.
//0 <= y < surface->h.
inline void DrawSpan(
		SDL_Surface *surface,
		SDL_Surface *tex,
		int x1,
		int x2,
		int y){

	int yorig = y;
	int x1orig = x1;

	y = y - surface->h/2; //Bottom of screen is positive
	x1 = x1 - surface->w/2; //Right of screen is positive
	x2 = x2 - surface->w/2; //Right of screen is positive

	//(tx, ty) = (h/y)(c*d + x*n) + pos
	float ty = (dir.hpos/y)*(dir.cpos*dir.py + x1*dir.ny) + dir.ypos;
	float tx = (dir.hpos/y)*(dir.cpos*dir.px + x1*dir.nx) + dir.xpos;

	int count = x2 - x1;

	//(dx, dy) = (h/y)(n)
	float xfrac = (dir.hpos/y)*(dir.nx);
	float yfrac = (dir.hpos/y)*(dir.ny);

	//printf("(xfrac, yfrac) = (%lf,%lf)\n", xfrac, yfrac);

	//Assumes identical 32 bit pixel format

	Uint16 pitch = surface->pitch;

	Uint32 *dst = ((Uint32*)surface->pixels) + (pitch)*yorig + x1orig*4;

	Uint32 *src = NULL;


	int inttx, intty;

	while(count >= 0){


		//Cast to integers
		inttx = tx;
		intty = ty;
		
		
		 /*remainder by 64 division
		(this is what DOOM did, it looks like!
		Only they used the actual decimal 63 instead of 3F.
		
		Description: (this is pretty genius of Carmack)

		As we know, a division by 64 is a shift right by 6 bits.
		And a multiplication by 64 is a shift left by 6 bits.

		So, the remainder of a division, which would be expressed by
		x = x - 64*(x/64)
		
		can be converted into
		x = x - (x >> 6 << 6)
		Now, since we know that the lower 6 bits have no choice but to
		be 0, we can optimize this by doing the following:
		x = x - (x & 0xFFFFFFC0)
		
		which will subtract x by itself without it's lower 6 bits.

		But wait, we can optimize this further.  When we're subtracting
		x by itself with it's lower 6 bits set to 0, all we're getting
		is...the lower 6 bits.  So:

		x = x & 0x3F

		now we have a remainder division, valid at least for
		positive numbers.

		But, that's not too useful to us, is it?  I mean, we have negative

		numbers in our texture coordinates.  Take (-1, 0), for instance.
		What do we do?  We'd like our result to be (63, 0) on the texture,
		because the texture is aligned to the XY axis and starting at
		(0,0).  Therefore we count from the right for the texture instead
		of the left, for negative numbers.  We, in other words, add 64
		to the negative numbers.  But this is inefficient.  We'd have to
		do a branch and add, like such:
		
		if (coord < 0) coord += 64;

		That's no fun. But, it turns out that the and by 0x3F works even for
		negative numbers!  For our purposes, anyway.
		
		Here's why:

		If inttx is negative, because of the way the two's
		complement works, the more negative you go, the less
		the 'positive' part of the number will be (the part of the
		number without the sign bit).  Example:
		-1 = 0xFFFFFFFF -> translates to 0x7FFFFFF with the sign
		bit off
		-2 = 0xFFFFFFFE -> translates to 0x7FFFFFE, a DECREASE from
		0x7FFFFFFF.

		Now, if we only take the first 6 bits, then
		this decrease is still present, AND we're counting backwards
		from 64!  Exactly what we want!
		
		Example:

		-5 = 0xFFFFFFFB
		0xFFFFFFFB & 0x0000003F = 0x00000003B = 59
	
		This operation therefore accepts any XY coordinate
		and determines where in the texture it lies.

		Only caveat: Texture is rendered upside down.  Could load them
		flipped to solve this, or subtract by 64 for positive case and 
		not for the negative case.
		*/
		intty = intty & 0x3F;
		inttx = inttx & 0x3F;
		
		src = ((Uint32*) tex->pixels) + intty*256 + inttx*4;

		*dst = *src;
		//Increment
		dst++;
		tx += xfrac;
		ty += yfrac;


		count--;
	}


}

#include <SDL.h>
#include <SDL_ttf.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
static constexpr float mouseSensitivity = 1;
#else

uint32_t game_running = 0;

void emscripten_cancel_main_loop() { game_running = 0; }

void emscripten_set_main_loop_arg(void (*fcn)(void *), void *const arg,
                                  uint32_t const fps, int const infinite_loop) {
  uint32_t const MPF =
      fps > 0 ? 1000 / fps : 1000 / 60; // milliseconds per frame

  // Only render a frame if at least the MPF has been passed in time
  // If the  time has not been passed, pop and process one event from
  // the event queue.  So, for fast computers, many events will be processed
  // in a single frame.  But for slow computers, only one event will be
  // popped and processed in a single frame.  Beneficial because on a slow
  // computer, if all events were popped at once, if the W key were tapped
  // for instance, it would add to the ydiff variable and subtract from it
  // before MovePlayer would have a chance to do anything.  In this case,
  // the player might move too far, but at least the player moved.

  game_running = 1;
  while (game_running) {
    uint32_t const startTime = SDL_GetTicks();
    fcn(arg);
    uint32_t const endTime = SDL_GetTicks();

    uint32_t const timeDiff = endTime - startTime;

    if (timeDiff < MPF) {
      SDL_Delay(
          MPF -
          timeDiff); // Sleep for the remainder of the time to maintain FPS
    }
  }
}

static constexpr float mouseSensitivity = 0.1;
#endif

struct GameResources {
  SDL_Window *window;
  SDL_Renderer *renderer;
  SDL_Texture *texture;
  SDL_Surface *display;
  SDL_Surface *floor;
  SDL_Surface *skyTranspose;
  TTF_Font *font;
  uint32_t windowID;
};

void FreeGameResources(GameResources *res) {
  TTF_CloseFont(res->font);
  SDL_FreeSurface(res->floor);
  SDL_FreeSurface(res->skyTranspose);
  SDL_FreeSurface(res->display);
  SDL_DestroyTexture(res->texture);
  SDL_DestroyRenderer(res->renderer);
  SDL_DestroyWindow(res->window);
}

// Render properties
struct render {
  uint16_t xres; // X resolution
  uint16_t yres; // Y resolution
} render = {
    640, 480}; // Defaults: 640x480 at 35fps native (DOOM), vsync for emscripten

// Key states
struct movement {
  int xdiff; // Whether A(-1) or D(1) is down
  int ydiff; // Whether W(1) or S(-1) is down
  int hdiff; // Whether T(1) or G(-1) is down
  int cdiff; // Whether Y(-1) or H(1) is down
} move = {};

// Player location, direction, and speed
struct direction {
  // Direction vector
  float px;
  float py;

  // Normal vector (must point east in screenspace)
  float nx;
  float ny;

  // Player angle from X axis
  float theta;

  // Player position
  float xpos;
  float ypos;

  // Camera properties
  float hpos; // Height
  float cpos; // Distance from lens

  // Player speed
  float speed; // Units per millisecond
} dir = {1.0,  0.0,  0.0,   -1.0,   0.0f,
         0.0f, 0.0f, 80.0f, 400.0f, 192.0f / 1000.0f};

void MovePlayer(int timediff) {

  dir.px = std::cos((dir.theta / 180) * M_PI);
  dir.py = std::sin((dir.theta / 180) * M_PI);

  dir.nx = dir.py;
  dir.ny = -dir.px;

  // Travel along normal
  dir.xpos += move.xdiff * dir.speed * timediff * dir.nx;
  dir.ypos += move.xdiff * dir.speed * timediff * dir.ny;

  // Travel along direction vector
  dir.xpos += move.ydiff * dir.speed * timediff * dir.px;
  dir.ypos += move.ydiff * dir.speed * timediff * dir.py;

  dir.hpos += move.hdiff * dir.speed * timediff;
  dir.cpos += move.cdiff * dir.speed * timediff;
}

int ProcessEvent(uint32_t windowID) {

  char e = 0;

  static SDL_Event event;

  while (SDL_PollEvent(&event)) {
    switch (event.type) {

    case SDL_WINDOWEVENT:
      if (event.window.windowID == windowID) {
        switch (event.window.event) {

          /*case SDL_WINDOWEVENT_SIZE_CHANGED:  {
          width = event.window.data1;
          height = event.window.data2;
          break;
          }*/

        case SDL_WINDOWEVENT_CLOSE:
          SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Window closed\n");
          event.type = SDL_QUIT;
          SDL_PushEvent(&event);
          break;

        default:
          break;
        }
      }
      break;

    case SDL_QUIT:
      SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "SDL_QUIT!\n");
      return 0; // 0 -- signal terminate program

    case SDL_KEYUP:
    case SDL_KEYDOWN:

      if (event.key.repeat != 0) {
        break;
      }

      // Because I'm lazy: e is positive or negative depending
      // on whether it's a keyup or keydown
      e = SDL_KEYDOWN == event.type ? 1 : -1;

      switch (event.key.keysym.sym) {
      case SDLK_w: // forward
        move.ydiff += e;
        break;
      case SDLK_s: // backward
        move.ydiff -= e;
        break;
      case SDLK_a: // left
        move.xdiff -= e;
        break;
      case SDLK_d: // right
        move.xdiff += e;
        break;
      case SDLK_t: // height up
        move.hdiff += e;
        break;
      case SDLK_g: // height down
        move.hdiff -= e;
        break;
      case SDLK_y:       // viewer-to-camera distance decrease (zoom out)
        move.cdiff -= e; // slower
        break;
      case SDLK_h:       // viewer-to-camera distance increase (zoom in)
        move.cdiff += e; // slower
      default:
        break;
      }
      break;
    case SDL_MOUSEMOTION:

      if (SDL_GetRelativeMouseMode() == SDL_TRUE) {
        // dir.theta += event.motion.xrel;
        // xrel: west negative, east positive
        // yrel: north negative, south positive

        // Use negative to map positive to west and negative to east (right
        // handed system)
        float const worldPixelAngle =
            180.0 *
            (std::atan2(float(event.motion.xrel) * mouseSensitivity, dir.cpos) /
             M_PI);
        dir.theta -= worldPixelAngle;
        // Ugly, but it works
        dir.theta /= 360;
        dir.theta = dir.theta - floor(dir.theta);
        dir.theta *= 360;
      }

      break;

    case SDL_MOUSEBUTTONUP:
      SDL_SetRelativeMouseMode(SDL_GetRelativeMouseMode() ? SDL_FALSE
                                                          : SDL_TRUE);
      break;
    }
  }
  return 1; // 1 - signal something processed
}

// Tex must be 64x64.
// 0 <= x1 < x <= surface->w.
// 0 <= y < surface->h.
void DrawSpan(SDL_Surface *surface, SDL_Surface *tex, int x1, int x2, int y) {

  uint32_t *dst = ((uint32_t *)surface->pixels) + (surface->w) * y + x1;

  int count = x2 - x1;

  y = y - surface->h / 2;   // South of screen is positive
  x1 = x1 - surface->w / 2; // West of screen is negative
  x2 = x2 - surface->w / 2; // East of screen is positive

  //(tx, ty) = (h/y)(c*d + x*n) + pos
  float ty = (dir.hpos / y) * (dir.cpos * dir.py + x1 * dir.ny) + dir.ypos;
  float tx = (dir.hpos / y) * (dir.cpos * dir.px + x1 * dir.nx) + dir.xpos;

  //(dx, dy) = (h/y)(n)
  float xfrac = (dir.hpos / y) * (dir.nx);
  float yfrac = (dir.hpos / y) * (dir.ny);

  // printf("(xfrac, yfrac) = (%lf,%lf)\n", xfrac, yfrac);

  // Assumes identical 32 bit pixel format

  // Uint16 pitch = surface->pitch;

  while (count >= 0) {

    /*remainder by 64 division
   (this is roughly what DOOM did
   Only they used the actual decimal 63 instead of 3F.

   Description:

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
   to the negative numbers.
   

   Fortunately, when casting to unsigned integers, it wraps from the highest
   number, giving us the desired behaviour.  So, first cast to int32_t to
   remove fractional part (floor) while preserving sign, then cast to uint32_t
   to get 2's complement semantics on all architectures, leading to the desired behaviour.
   Note: emscripten complains if a direct conversion to uint32_t is attempted and it's negative.

   Now, if we only take the first 6 bits, then
   this decrease is still present, AND we're counting backwards
   from 64!  Exactly what we want!
   

   Example:

   -5 (int) = 0xFFFFFFFB (uint32_t)
   0xFFFFFFFB & 0x0000003F = 0x00000003B = 59

   This operation therefore accepts any XY coordinate
   and determines where in the texture it lies.

   Only caveat: Texture is rendered upside down.  Could load them
   flipped to solve this, or subtract by 64 for positive case and
   not for the negative case.
   */
    uint32_t uintty = uint32_t(int32_t(ty)) & 0x3F;
    uint32_t uinttx = uint32_t(int32_t(tx)) & 0x3F;

    uint32_t *src = ((Uint32 *)tex->pixels) + uintty * 64 + uinttx;

    *dst = *src;
    // Increment
    dst++;
    tx += xfrac;
    ty += yfrac;

    --count;
  }
}

void DrawColumn(SDL_Surface *const display,
                SDL_Surface const *const texTranspose, size_t const dx,
                size_t const dy1, size_t const dy2, size_t const tx,
                size_t const ty1, size_t const ty2) {
  uint32_t *displayPixels = (uint32_t *)display->pixels;
  uint32_t *texTPixels = (uint32_t *)texTranspose->pixels;

  float const yFrac = float(ty2 - ty1) / float(dy2 - dy1);

  float ti = (float)ty1;
  for (size_t di = dy1; di < dy2; ++di) {
    *(displayPixels + (display->w) * di + dx) =
        *(texTPixels + (texTranspose->w) * tx + (size_t)ti);

    ti += yFrac;
  }
}

void Draw(GameResources *res, uint32_t const frameTime[2]) {

  SDL_Surface *display = res->display;

  // Render sky
  for (size_t i = 0; i < (size_t)display->w; ++i) {
    // Treat sky as being projected in a cylinder, with deformation happening
    // around the edges of the screen.
    // The sky is wrapped around the cylinder 4 times (every 90 degrees the
    // image repeats)

    // Recall: left side of screen corresponds to +ve
    float worldPixelAngle =
        dir.theta +
        180.0 * (std::atan2((display->w / 2.0) - i, dir.cpos) / M_PI);
    if (worldPixelAngle < 0.0) {
      worldPixelAngle += 360.0;
    } else if (worldPixelAngle > 360.0) // UGLY, but it works
    {
      worldPixelAngle /= 360.0;
      worldPixelAngle -= std::floor(worldPixelAngle);
      worldPixelAngle *= 360.0;
    }

    // float const mirroredCoord = worldPixelAngle > 180.0 ? (1.0 -
    // (worldPixelAngle - 180.0) / 180.0) : (worldPixelAngle / 180.0);

    float const finalCoord =
        (worldPixelAngle / 90.0) - std::floor(worldPixelAngle / 90.0);

    DrawColumn(display, res->skyTranspose, i, 0, display->h / 2 + 1,
               finalCoord * res->skyTranspose->h, 0, res->skyTranspose->w);
  }

  // Render floor
  for (int i = display->h / 2 + 1; i < display->h; ++i) {
    DrawSpan(display, res->floor, 0, render.xres - 1, i);
  }

  // Render FPS
  static char frameTimeString[128];

  if (frameTime[0] != 0 || frameTime[1] != 0) {
    static SDL_Color const colour = {255, 255, 0, 255};

    SDL_snprintf(frameTimeString, sizeof(frameTimeString),
                 "Previous frame time: %ums", frameTime[1] - frameTime[0]);

    SDL_Surface *fpsGauge =
        TTF_RenderText_Blended(res->font, frameTimeString, colour);

    SDL_BlitSurface(fpsGauge, NULL, display, NULL);

    SDL_FreeSurface(fpsGauge);
  }
}

void GameLoop(void *const arg) {

  GameResources *const res = (GameResources *const)arg;

  static uint32_t prevFrameTime[2] = {};

  prevFrameTime[0] = prevFrameTime[1];
  prevFrameTime[1] = SDL_GetTicks();

  MovePlayer(prevFrameTime[1] - prevFrameTime[0]);

  if (ProcessEvent(res->windowID) == 0) {
    FreeGameResources(res);
    emscripten_cancel_main_loop();
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Exiting!\n");
    SDL_Quit();
    std::exit(0); // Reason for not plain returning: emscripten won't call
                  // global destructors when main or the main loop exits.
  }

  Draw(res, prevFrameTime);
  SDL_UpdateTexture(res->texture, NULL, res->display->pixels,
                    res->display->pitch);
  // Now draw this directly to the window
  SDL_RenderClear(res->renderer);
  SDL_RenderCopy(res->renderer, res->texture, NULL, NULL);
  // And ask the windowing system to redraw the window
  SDL_RenderPresent(res->renderer);

  // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Prev frame time: %u, current
  // present: %u", prevFrameTime[1] - prevFrameTime[0], SDL_GetTicks() -
  // prevFrameTime[1]);

  // MovePlayer(SDL_GetTicks() - prevFrameTime[1]);
  // MovePlayer(prevFrameTime[1] - prevFrameTime[0]);
  // MovePlayer(16);
}

int main(int argc, char *argv[]) {

  // VERY IMPORTANT: Ensure SDL2 is initialized
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_EVENTS) < 0) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                 "could not initialize sdl2: %s\n", SDL_GetError());
    return 1;
  }

  // VERY IMPORTANT: if using text in your program, ensure SDL2_ttf library is
  // initialized
  if (TTF_Init() < 0) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                 "could not initialize SDL2_ttf: %s\n", TTF_GetError());
  }

  GameResources res;

  // This creates the actual window in which graphics are displayed
  res.window = SDL_CreateWindow("Floor - SDL2 version", SDL_WINDOWPOS_UNDEFINED,
                                SDL_WINDOWPOS_UNDEFINED, render.xres,
                                render.yres, SDL_WINDOW_SHOWN);

  if (res.window == NULL) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not create window!\n");
    return 1;
  }

  res.windowID = SDL_GetWindowID(res.window);

  res.renderer = SDL_CreateRenderer(res.window, -1,
                                    0); // don't force hardware or software
                                        // accel -- let SDL2 choose what's best

  if (res.renderer == NULL) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                 "could not create renderer: %s\n", SDL_GetError());
    return 1;
  }

  res.display = SDL_CreateRGBSurfaceWithFormat(0, render.xres, render.yres, 32,
                                               SDL_PIXELFORMAT_RGBA8888);
  if (res.display == NULL) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create surface: %s\n",
                 SDL_GetError());
    return 1;
  }

  res.texture =
      SDL_CreateTexture(res.renderer, SDL_PIXELFORMAT_RGBA8888,
                        SDL_TEXTUREACCESS_STREAMING, render.xres, render.yres);

  if (res.texture == NULL) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create texture: %s\n",
                 SDL_GetError());
    return 1;
  }

  // Just in case you need text:
  // load iosevka-regular.ttf at a large size into font
  res.font = TTF_OpenFont("iosevka-regular.ttf", 16);
  if (!res.font) {
    SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "TTF_OpenFont: %s\n",
                 TTF_GetError());
    return 1;
  }

  {
    SDL_Surface *tmp = SDL_LoadBMP("floor.bmp");

    if (tmp == NULL) {
      SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not load floor.bmp!\n");
      return 1;
    }

    res.floor = SDL_ConvertSurface(tmp, res.display->format, 0);

    SDL_FreeSurface(tmp);
  }

  {
    SDL_Surface *tmp = SDL_LoadBMP("sky.bmp");

    if (tmp == NULL) {
      SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not load sky.bmp!\n");
      return 1;
    }

    res.skyTranspose = SDL_ConvertSurface(tmp, res.display->format, 0);

    SDL_FreeSurface(tmp);
  }

  emscripten_set_main_loop_arg(GameLoop, &res, 0, 1);

  return 0;
}

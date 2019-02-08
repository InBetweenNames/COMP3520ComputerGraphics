#include <SDL.h>
#include <SDL_ttf.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <stdexcept>

#define M_PI_F 3.14159265358979323846264338327950288f

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
static constexpr float mouseSensitivity = 1;
#else

uint32_t game_running = 0;

void emscripten_cancel_main_loop() { game_running = 0; }

void emscripten_set_main_loop_arg(void (*fcn)(void*), void* const arg, uint32_t const fps, int const infinite_loop)
{
    uint32_t const MPF = fps > 0 ? 1000 / fps : 1000 / 60; // milliseconds per frame

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
    while (game_running)
    {
        uint32_t const startTime = SDL_GetTicks();
        fcn(arg);
        uint32_t const endTime = SDL_GetTicks();

        uint32_t const timeDiff = endTime - startTime;

        if (timeDiff < MPF)
        {
            SDL_Delay(MPF - timeDiff); // Sleep for the remainder of the time to maintain FPS
        }
    }
}

static constexpr float mouseSensitivity = 0.1f;
#endif

struct GameResources
{
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    SDL_Surface* display;
    SDL_Surface* floor;
    SDL_Surface* skyTranspose;
    SDL_Surface* wallTranspose;
    SDL_Surface* columnSprite;
    SDL_Surface* healthSprite;
    SDL_Surface* vialSprite;
    SDL_Surface* doomGuy[8];
    TTF_Font* font;
    uint32_t windowID;
};

void FreeGameResources(GameResources* res)
{
    TTF_CloseFont(res->font);
    SDL_FreeSurface(res->floor);
    SDL_FreeSurface(res->skyTranspose);
    SDL_FreeSurface(res->wallTranspose);
    SDL_FreeSurface(res->display);
    SDL_FreeSurface(res->columnSprite);
    SDL_FreeSurface(res->healthSprite);
    SDL_FreeSurface(res->vialSprite);
    for (size_t i = 0; i < 8; i++)
    {
        SDL_FreeSurface(res->doomGuy[i]);
    }
    SDL_DestroyTexture(res->texture);
    SDL_DestroyRenderer(res->renderer);
    SDL_DestroyWindow(res->window);
}

// Render properties
struct render
{
    uint16_t xres;     // X resolution
    uint16_t yres;     // Y resolution
} render = {640, 480}; // Defaults: 640x480 at 35fps native (DOOM), vsync for emscripten

// Key states
struct movement
{
    int xdiff; // Whether A(-1) or D(1) is down
    int ydiff; // Whether W(1) or S(-1) is down
    int hdiff; // Whether T(1) or G(-1) is down
    int cdiff; // Whether Y(-1) or H(1) is down
} move = {};

// Player location, direction, and speed
struct direction
{
    // Direction vector
    Eigen::Vector2f p;

    // Normal vector (must point east in screenspace)
    Eigen::Vector2f n;

    // Player angle from X axis
    float theta;

    // Player position
    Eigen::Vector2f pos;

    // Camera properties
    float hpos; // Height
    float cpos; // Distance from lens

    // Player speed
    float speed; // Units per millisecond
} dir = {{1.0f, 0.0f}, {0.0f, -1.0f}, 0.0f, {0.0f, 0.0f}, 48.0f, 400.0f, 192.0f / 1000.0f};

void MovePlayer(int timediff)
{
    dir.p.x() = std::cos((dir.theta / 180.0f) * M_PI_F);
    dir.p.y() = std::sin((dir.theta / 180.0f) * M_PI_F);

    dir.n.x() = dir.p.y();
    dir.n.y() = -dir.p.x();

    // Travel along normal
    dir.pos += dir.speed * timediff * move.xdiff * dir.n;

    // Travel along direction vector
    dir.pos += dir.speed * timediff * move.ydiff * dir.p;

    dir.hpos += move.hdiff * dir.speed * timediff;
    dir.cpos += move.cdiff * dir.speed * timediff;
}

int ProcessEvent(uint32_t windowID)
{
    char e = 0;

    static SDL_Event event;

    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_WINDOWEVENT:
            if (event.window.windowID == windowID)
            {
                switch (event.window.event)
                {
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

            if (event.key.repeat != 0)
            {
                break;
            }

            // Because I'm lazy: e is positive or negative depending
            // on whether it's a keyup or keydown
            e = SDL_KEYDOWN == event.type ? 1 : -1;

            switch (event.key.keysym.sym)
            {
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
            case SDLK_y:         // viewer-to-camera distance decrease (zoom out)
                move.cdiff -= e; // slower
                break;
            case SDLK_h:         // viewer-to-camera distance increase (zoom in)
                move.cdiff += e; // slower
                break;
            default:
                break;
            }
            break;
        case SDL_MOUSEMOTION:

            if (SDL_GetRelativeMouseMode() == SDL_TRUE)
            {
                // dir.theta += event.motion.xrel;
                // xrel: west negative, east positive
                // yrel: north negative, south positive

                // Use negative to map positive to west and negative to east (right
                // handed system)
                float const worldPixelAngle =
                    180.0f * (std::atan2(float(event.motion.xrel) * mouseSensitivity, dir.cpos) / M_PI_F);
                dir.theta -= worldPixelAngle;
                // Ugly, but it works
                dir.theta /= 360.0f;
                dir.theta = dir.theta - floor(dir.theta);
                dir.theta *= 360.0f;
            }

            break;

        case SDL_MOUSEBUTTONUP:
            SDL_SetRelativeMouseMode(SDL_GetRelativeMouseMode() ? SDL_FALSE : SDL_TRUE);
            break;
        }
    }
    return 1; // 1 - signal something processed
}

// Tex must be 64x64.
// 0 <= x1 < x <= surface->w.
// 0 <= y < surface->h.
void DrawSpan(SDL_Surface* surface, SDL_Surface* tex, int x1, int x2, int y)
{
    uint32_t* dst = ((uint32_t*)surface->pixels) + (surface->w) * y + x1;

    int count = x2 - x1;

    y = y - surface->h / 2;   // South of screen is positive
    x1 = x1 - surface->w / 2; // West of screen is negative
    x2 = x2 - surface->w / 2; // East of screen is positive

    //(tx, ty) = (h/y)(c*d + x*n) + pos
    // float ty = (dir.hpos / y) * (dir.cpos * dir.p.y() + x1 * dir.n.y()) + dir.pos.y();
    // float tx = (dir.hpos / y) * (dir.cpos * dir.p.x() + x1 * dir.n.x()) + dir.pos.x();
    Eigen::Vector2f t = (dir.hpos / y) * (dir.cpos * dir.p + x1 * dir.n) + dir.pos;

    //(dx, dy) = (h/y)(n)
    // float xfrac = (dir.hpos / y) * (dir.n.x());
    // float yfrac = (dir.hpos / y) * (dir.n.y());
    Eigen::Vector2f frac = dir.hpos * dir.n / y;

    // printf("(xfrac, yfrac) = (%lf,%lf)\n", xfrac, yfrac);

    // Assumes identical 32 bit pixel format

    // Uint16 pitch = surface->pitch;

    while (count >= 0)
    {
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
    to get 2's complement semantics on all architectures, leading to the desired
    behaviour. Note: emscripten complains if a direct conversion to uint32_t is
    attempted and it's negative.

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
        uint32_t uintty = uint32_t(int32_t(t.y())) & 0x3F;
        uint32_t uinttx = uint32_t(int32_t(t.x())) & 0x3F;

        uint32_t* src = ((uint32_t*)tex->pixels) + uintty * 64 + uinttx;

        *dst = *src;
        // Increment
        dst++;
        t += frac;
        // tx += xfrac;
        // ty += yfrac;

        --count;
    }
}

void DrawColumn(SDL_Surface* const display, SDL_Surface const* const texTranspose, size_t const dx, size_t const dy1,
                size_t const dy2, size_t const tx, size_t const ty1, size_t const ty2)
{
    uint32_t* displayPixels = (uint32_t*)display->pixels;
    uint32_t* texTPixels = (uint32_t*)texTranspose->pixels;

    float const yFrac = float(ty2 - ty1) / float(dy2 - dy1);

    float ti = (float)ty1;
    for (size_t di = dy1; di < dy2; ++di)
    {
        *(displayPixels + (display->w) * di + dx) = *(texTPixels + (texTranspose->w) * tx + (size_t)ti);

        ti += yFrac;
    }
}

void RenderSprite(SDL_Surface* display, SDL_Surface* const sprite, float const x, float const z,
                  int const hOffset) noexcept
{
    Eigen::Matrix3f viewInverse = Eigen::Matrix3f::Identity();
    viewInverse.topLeftCorner<1, 2>() = dir.n.transpose();
    viewInverse.block<1, 2>(1, 0) = dir.p.transpose();
    viewInverse.topRightCorner<2, 1>() = viewInverse.topLeftCorner<2, 2>() * (-dir.pos);

    Eigen::Vector3f viewSpritePos = viewInverse * Eigen::Vector3f{x, z, 1};

    auto const halfWidth = sprite->w / 2;

    auto const sCentre = viewSpritePos.x();
    auto const sMin = sCentre - halfWidth;
    auto const sMax = sCentre + halfWidth;

    auto const d = viewSpritePos.y();
    auto const c = dir.cpos;

    // centre_x = s/d * c

    // auto const xCentre = (s / d) * c;
    auto const hTop = float(sprite->h + hOffset) - dir.hpos;
    auto const hBot = float(hOffset) - dir.hpos;

    auto const yTop = (hTop * c) / d;
    auto const yBot = (hBot * c) / d;

    auto const xMin = (sMin * c) / d;
    auto const xMax = (sMax * c) / d;

    int const screenTop = display->h / 2 - yTop;
    // int const screenBot = display->h / 2 - yBot;
    int const screenXMin = display->w / 2 + xMin;
    // int const screenXMax = display->w / 2 + xMax;

    // TODO: replace this with custom blitting routine
    SDL_Rect finalRect;
    finalRect.h = yTop - yBot;
    finalRect.w = xMax - xMin;
    finalRect.x = screenXMin;
    finalRect.y = screenTop;

    SDL_BlitScaled(sprite, nullptr, display, &finalRect);
}

void Draw(GameResources* res, uint32_t const frameTime[2])
{
    SDL_Surface* display = res->display;

    // Render sky
    // TODO: sky texture is being rendered flipped possibly.  Need to investigate.
    for (size_t i = 0; i < (size_t)display->w; ++i)
    {
        // Treat sky as being projected in a cylinder, with deformation happening
        // around the edges of the screen.
        // The sky is wrapped around the cylinder 4 times (every 90 degrees the
        // image repeats)

        // Recall: left side of screen corresponds to +ve
        float worldPixelAngle = dir.theta + 180.0f * (std::atan2((display->w / 2.0f) - i, dir.cpos) / M_PI_F);
        if (worldPixelAngle < 0.0f)
        {
            worldPixelAngle += 360.0f;
        }
        else if (worldPixelAngle > 360.0f)
        { // UGLY, but it works
            worldPixelAngle /= 360.0f;
            worldPixelAngle -= std::floor(worldPixelAngle);
            worldPixelAngle *= 360.0f;
        }

        // float const mirroredCoord = worldPixelAngle > 180.0f ? (1.0f -
        // (worldPixelAngle - 180.0f) / 180.0f) : (worldPixelAngle / 180.0f);

        float const finalCoord = (worldPixelAngle / 90.0f) - std::floor(worldPixelAngle / 90.0f);

        DrawColumn(display, res->skyTranspose, i, 0, display->h / 2 + 1,
                   size_t(finalCoord * float(res->skyTranspose->h)), 0, res->skyTranspose->w);
    }

    // Render floor
    for (int i = display->h / 2 + 1; i < display->h; ++i)
    {
        DrawSpan(display, res->floor, 0, render.xres - 1, i);
    }

    // Render FPS
    static char frameTimeString[128];

    if (frameTime[0] != 0 || frameTime[1] != 0)
    {
        static SDL_Color const colour = {255, 255, 0, 255};

        SDL_snprintf(frameTimeString, sizeof(frameTimeString), "Previous frame time: %ums",
                     frameTime[1] - frameTime[0]);

        SDL_Surface* fpsGauge = TTF_RenderText_Blended(res->font, frameTimeString, colour);

        SDL_BlitSurface(fpsGauge, nullptr, display, nullptr);

        SDL_FreeSurface(fpsGauge);
    }

    // Render sprites

    // Objects
    RenderSprite(display, res->columnSprite, 500, 0, 0);
    RenderSprite(display, res->vialSprite, 500, 100, 0);
    RenderSprite(display, res->healthSprite, 500, 200, 0);
    RenderSprite(display, res->columnSprite, 500, 300, 0);

    // Doomguy
    // Assume doomguy is facing [-1 0]
    Eigen::Vector2f doomGuyOrient{-1, 0};
    Eigen::Vector2f playerToDoomGuy = Eigen::Vector2f{550, 50} - dir.pos;
    auto const dot = playerToDoomGuy.dot(doomGuyOrient);
    auto const det = playerToDoomGuy(0) * doomGuyOrient(1) - playerToDoomGuy(1) * doomGuyOrient(0);
    auto angle = std::atan2(det, dot) - (M_PI_F / 8);

    /*if (angle < 0)
    {
        angle += 2 * M_PI_F;
    }*/

    int spriteSelect = 4 + std::floor((angle) / (M_PI_F / 4));
    if (spriteSelect >= 8)
    {
        spriteSelect = 0;
    }
    if (spriteSelect < 0)
    {
        spriteSelect = 7;
    }

    // TODO: figure out the actual transformation
    RenderSprite(display, res->doomGuy[7 - spriteSelect], 550, 50, 0);
}

void GameLoop(void* const arg)
{
    GameResources* const res = (GameResources* const)arg;

    static uint32_t prevFrameTime[2] = {};

    prevFrameTime[0] = prevFrameTime[1];
    prevFrameTime[1] = SDL_GetTicks();

    if (ProcessEvent(res->windowID) == 0)
    {
        FreeGameResources(res);
        emscripten_cancel_main_loop();
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Exiting!\n");
        SDL_Quit();
        std::exit(0); // Reason for not plain returning: emscripten won't call
                      // global destructors when main or the main loop exits.
    }

    MovePlayer(prevFrameTime[1] - prevFrameTime[0]);

    Draw(res, prevFrameTime);
    SDL_UpdateTexture(res->texture, nullptr, res->display->pixels, res->display->pitch);
    // Now draw this directly to the window
    SDL_RenderClear(res->renderer);
    SDL_RenderCopy(res->renderer, res->texture, nullptr, nullptr);
    // And ask the windowing system to redraw the window
    SDL_RenderPresent(res->renderer);

    // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Prev frame time: %u, current
    // present: %u", prevFrameTime[1] - prevFrameTime[0], SDL_GetTicks() -
    // prevFrameTime[1]);

    // MovePlayer(SDL_GetTicks() - prevFrameTime[1]);
    // MovePlayer(prevFrameTime[1] - prevFrameTime[0]);
    // MovePlayer(16);
}

SDL_Surface* LoadTexture(char const* filename, SDL_PixelFormat const* const dstformat)
{
    SDL_Surface* tmp = SDL_LoadBMP(filename);

    if (tmp == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not load %s!\n", filename);
        throw std::runtime_error("Couldn't load texture");
    }

    SDL_Surface* result = SDL_ConvertSurface(tmp, dstformat, 0);
    if (result == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not convert surface for %s!\n", filename);
        throw std::runtime_error("Couldn't load texture");
    }

    SDL_FreeSurface(tmp);

    return result;
}

int main(int argc, char* argv[])
{
    // VERY IMPORTANT: Ensure SDL2 is initialized
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_EVENTS) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not initialize sdl2: %s\n", SDL_GetError());
        return 1;
    }

    // VERY IMPORTANT: if using text in your program, ensure SDL2_ttf library is
    // initialized
    if (TTF_Init() < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not initialize SDL2_ttf: %s\n", TTF_GetError());
    }

    GameResources res;

    // This creates the actual window in which graphics are displayed
    res.window = SDL_CreateWindow("Floor - SDL2 version", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, render.xres,
                                  render.yres, SDL_WINDOW_SHOWN);

    if (res.window == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Could not create window!\n");
        return 1;
    }

    res.windowID = SDL_GetWindowID(res.window);

    res.renderer = SDL_CreateRenderer(res.window, -1,
                                      0); // don't force hardware or software
    // accel -- let SDL2 choose what's best

    if (res.renderer == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create renderer: %s\n", SDL_GetError());
        return 1;
    }

    res.display = SDL_CreateRGBSurfaceWithFormat(0, render.xres, render.yres, 32, SDL_PIXELFORMAT_RGBA8888);
    if (res.display == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create surface: %s\n", SDL_GetError());
        return 1;
    }

    res.texture = SDL_CreateTexture(res.renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, render.xres,
                                    render.yres);

    if (res.texture == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create texture: %s\n", SDL_GetError());
        return 1;
    }

    // Just in case you need text:
    // load iosevka-regular.ttf at a large size into font
    res.font = TTF_OpenFont("../assets/iosevka-regular.ttf", 16);
    if (res.font == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "TTF_OpenFont: %s\n", TTF_GetError());
        return 1;
    }

    res.floor = LoadTexture("../assets/floor.bmp", res.display->format);
    res.skyTranspose = LoadTexture("../assets/sky.bmp", res.display->format);
    res.wallTranspose = LoadTexture("../assets/wall.bmp", res.display->format);
    res.healthSprite = LoadTexture("../assets/health.bmp", res.display->format);
    res.vialSprite = LoadTexture("../assets/vial.bmp", res.display->format);
    res.columnSprite = LoadTexture("../assets/column.bmp", res.display->format);

    char pathBuf[128];
    for (size_t i = 0; i < 8; i++)
    {
        snprintf(pathBuf, 128, "../assets/playa%zu.bmp", i + 1);
        res.doomGuy[i] = LoadTexture(pathBuf, res.display->format);
    }

    if (SDL_SetSurfaceBlendMode(res.display, SDL_BLENDMODE_BLEND) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_SetSurfaceBlendMode: %s\n", SDL_GetError());
        return 1;
    }

    emscripten_set_main_loop_arg(GameLoop, &res, 0, 1);

    return 0;
}

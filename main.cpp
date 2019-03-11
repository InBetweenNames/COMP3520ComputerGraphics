#pragma warning(push, 0)
#pragma warning(disable : ALL_CODE_ANALYSIS_WARNINGS)
#include <SDL.h>
#include <SDL_ttf.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <stdexcept>
#pragma warning(pop)

#define M_PI_F 3.14159265358979323846264338327950288f

template <typename T> using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
static constexpr float mouseSensitivity = 1;
#else

inline uint32_t game_running = 0;

void emscripten_cancel_main_loop() { game_running = 0; }

void emscripten_set_main_loop_arg(void (*fcn)(void*), void* const arg, uint32_t const fps, int const)
{
    uint32_t const MPF = fps > 0 ? 1000 / fps : 1000 / 60; // milliseconds per frame

    // Only render a frame if at least the MPF has been passed in time
    // If the  time has not been passed, pop and process one event from
    // the event queue.  So, for fast computers, many events will be processed
    // in a single frame.  But for slow computers, only one event will be
    // popped and processed in a single frame.  Beneficial because on a slow
    // computer, if all events were popped at once, if the W key were tapped
    // for instance, it would add to the zdiff variable and subtract from it
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

struct Entity
{
    Entity(std::string_view const _name, SDL_Surface* const _texture, Eigen::Vector3f const& _pos) noexcept
        : name{_name}, texture{_texture}, pos{_pos}
    {
    }

    std::string name;
    SDL_Surface* texture;
    Eigen::Vector3f pos; // x, y, h

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Player location, direction, and speed
struct direction
{
    direction() : pos{0.0f, 48.0f, 0.0f}, theta{M_PI_F / 2}, c{400.0f}, speed{192.0f / 1000.0f} {}

    // Direction vector
    // Eigen::Vector2f p;

    // Normal vector (must point east in screenspace)
    // Eigen::Vector2f n;

    // Player position
    Eigen::Vector3f pos;

    // Player angle from X axis
    float theta;

    // Camera properties
    float c; // Distance from lens (TODO: move to projection matrix)

    // Player speed
    float speed; // Units per millisecond
};

// Key states
struct movement
{
    float xdiff; // Whether A(-1) or D(1) is down
    float zdiff; // Whether W(1) or S(-1) is down
    float hdiff; // Whether T(1) or G(-1) is down
    float cdiff; // Whether Y(-1) or H(1) is down
};

// Render properties
// Defaults: 640x480 at 35fps native (DOOM), vsync for emscripten

struct GameResources
{
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    SDL_Surface* display;
    SDL_Surface* floor;
    SDL_Surface* skyTranspose;
    SDL_Surface* wallFrontTranspose;
    SDL_Surface* wallBackTranspose;
    SDL_Surface* columnSprite;
    SDL_Surface* healthSprite;
    SDL_Surface* vialSprite;
    SDL_Surface* doomGuy[8];
    TTF_Font* font;

    AlignedVector<Entity> entities;

    direction dir;
    movement move = {};

    Eigen::Matrix4f viewInverse = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    uint32_t windowID;

    ~GameResources() noexcept
    {
        TTF_CloseFont(font);
        SDL_FreeSurface(floor);
        SDL_FreeSurface(skyTranspose);
        SDL_FreeSurface(wallFrontTranspose);
        SDL_FreeSurface(wallBackTranspose);
        SDL_FreeSurface(display);
        SDL_FreeSurface(columnSprite);
        SDL_FreeSurface(healthSprite);
        SDL_FreeSurface(vialSprite);
        for (size_t i = 0; i < 8; i++)
        {
            SDL_FreeSurface(doomGuy[i]);
        }
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void MovePlayer(direction& dir, Eigen::Matrix4f const& viewInverse, movement const& move, uint32_t const timediff)
{
    // Position vector
    dir.pos +=
        dir.speed * timediff * viewInverse.topLeftCorner<3, 3>() * Eigen::Vector3f{move.xdiff, move.hdiff, move.zdiff};

    // Adjust focal length of camera (TODO: move to projection matrix)
    dir.c += move.cdiff * dir.speed * timediff;
}

void UpdateViewMatrices(direction const& dir, Eigen::Matrix4f& viewInverse, Eigen::Matrix4f& view)
{
    // viewInverse already initialized to Identity on startup

    // Upper left corner is a rotation matrix -- playing field is the XZ plane
    viewInverse.topLeftCorner<3, 3>() = Eigen::AngleAxis<float>{dir.theta, Eigen::Vector3f{0, 1, 0}}.toRotationMatrix();

    // View inverse X should be going in scanline direction (left-handed)
    viewInverse.col(0) *= -1;

    // dir.p.x() = std::cos(dir.theta);
    // dir.p.y() = std::sin(dir.theta);

    // dir.n.x() = dir.p.y();
    // dir.n.y() = -dir.p.x();

    viewInverse.topRightCorner<3, 1>() = dir.pos;

    // Travel along normal
    // dir.pos.topRows<2>() += dir.speed * timediff * move.xdiff * dir.n;

    // Travel along direction vector
    // dir.pos.topRows<2>() += dir.speed * timediff * move.zdiff * dir.p;

    // Move height of camera
    // dir.pos(2) += move.hdiff * dir.speed * timediff;

    // Adjust focal length of camera (TODO: move to projection matrix)
    // dir.c += move.cdiff * dir.speed * timediff;

    // view = Eigen::Matrix4f::Identity(); // not required as view was already
    // initialized to I float inverseC = 1.0f / viewInverse(3, 3);

    view.topLeftCorner<3, 3>() = viewInverse.topLeftCorner<3, 3>().transpose();
    view.topRightCorner<3, 1>() = -view.topLeftCorner<3, 3>() * dir.pos; // * inverseC;
                                                                         // view(3, 3) = inverseC;
}

int ProcessEvent(direction& dir, movement& move, uint32_t windowID)
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
                move.zdiff += e;
                break;
            case SDLK_s: // backward
                move.zdiff -= e;
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
                float const worldPixelAngle = std::atan2(float(event.motion.xrel) * mouseSensitivity, dir.c);
                dir.theta -= worldPixelAngle;
                // Ugly, but it works
                dir.theta /= 2 * M_PI_F;
                dir.theta = dir.theta - floor(dir.theta);
                dir.theta *= 2 * M_PI_F;
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
void DrawSpan(SDL_Surface* surface, SDL_Surface* tex, Eigen::Matrix4f const& viewInverse, float c, int x1, int x2,
              int y)
{
    uint32_t* dst = (static_cast<uint32_t*>(surface->pixels)) + (surface->w) * y + x1;

    int count = x2 - x1;

    y = y - surface->h / 2;   // South of screen is positive
    x1 = x1 - surface->w / 2; // West of screen is negative
    x2 = x2 - surface->w / 2; // East of screen is positive

    float const cameraHeight = viewInverse(1, 3);
    Eigen::Vector2f const directionVector = Eigen::Vector2f{viewInverse(0, 2), viewInverse(2, 2)};
    Eigen::Vector2f const normalVector = Eigen::Vector2f{viewInverse(0, 0), viewInverse(2, 0)};
    Eigen::Vector2f const position2D = Eigen::Vector2f{viewInverse(0, 3), viewInverse(2, 3)};

    //(tx, ty) = (h/y)(c*d + x*n) + pos
    // float ty = (dir.hpos / y) * (dir.c * dir.p.y() + x1 * dir.n.y()) +
    // dir.pos.y(); float tx = (dir.hpos / y) * (dir.c * dir.p.x() + x1 *
    // dir.n.x()) + dir.pos.x();
    Eigen::Vector2f t = (cameraHeight / y) * (c * directionVector + x1 * normalVector) + position2D;

    //(dx, dy) = (h/y)(n)
    // float xfrac = (dir.hpos / y) * (dir.n.x());
    // float yfrac = (dir.hpos / y) * (dir.n.y());
    Eigen::Vector2f const frac = cameraHeight * normalVector / y;

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
        size_t uintty = size_t(ptrdiff_t(t.y())) & 0x3F;
        size_t uinttx = size_t(ptrdiff_t(t.x())) & 0x3F;

        uint32_t* src = static_cast<uint32_t*>(tex->pixels) + uintty * 64 + uinttx;

        *dst = *src;
        // Increment
        dst++;
        t += frac;
        // tx += xfrac;
        // ty += yfrac;

        --count;
    }
}

void DrawColumn(SDL_Surface* const display, SDL_Surface const* const texTranspose, size_t const dx, ptrdiff_t const dy1,
                ptrdiff_t const dy2, float const tx, float const ty1, float const ty2)
{
    uint32_t* displayPixels = static_cast<uint32_t*>(display->pixels);
    uint32_t* texTPixels = static_cast<uint32_t*>(texTranspose->pixels);

    size_t const displayWidth = static_cast<size_t>(display->w);
    size_t const texTransposeWidth = static_cast<size_t>(texTranspose->w);
    float const texTransposeWidthF = static_cast<float>(texTranspose->w);
    size_t const texTransposeHeight = static_cast<size_t>(texTranspose->h);

    size_t const tMappedX =
        static_cast<size_t>(static_cast<ptrdiff_t>(tx * static_cast<float>(texTranspose->h))) % texTransposeHeight;

    float const yFrac = (ty2 - ty1) / float(dy2 - dy1);

    uint32_t* texTCol = texTPixels + tMappedX * texTransposeWidth;

    // size_t const mappedCurrTopY = static_cast<size_t>(std::clamp(currTopY, 0.0f, float(display->h)));
    // size_t const mappedCurrBotY = static_cast<size_t>(std::clamp(currBotY, 0.0f, float(display->h)));

    ptrdiff_t const clampDy1 = std::clamp(dy1, ptrdiff_t(0), static_cast<ptrdiff_t>(display->h));
    ptrdiff_t const clampDy2 = std::clamp(dy2, ptrdiff_t(0), static_cast<ptrdiff_t>(display->h));

    float ti = ty1 + (clampDy1 - dy1) * (ty2 - ty1) / (dy2 - dy1);

    for (ptrdiff_t di = clampDy1; di < clampDy2; ++di)
    {
        size_t const tMappedY =
            static_cast<size_t>(static_cast<ptrdiff_t>(ti * texTransposeWidthF)) % texTransposeWidth;

        *(displayPixels + displayWidth * di + dx) = *(texTCol + tMappedY);

        ti += yFrac;
    }
}

void RenderSprite(SDL_Surface* display, SDL_Surface* sprite, float const c, float const x, float const h,
                  float const z) noexcept
{

    auto const halfWidth = sprite->w / 2;

    auto const sCentre = x;
    auto const sMin = sCentre - halfWidth;
    auto const sMax = sCentre + halfWidth;

    auto const d = z;
    // auto const c = dir.c;

    // centre_x = s/d * c

    // auto const xCentre = (s / d) * c;
    auto const hTop = float(sprite->h) + h;
    auto const hBot = h;

    int const yTop = static_cast<int>((hTop * c) / d);
    int const yBot = static_cast<int>((hBot * c) / d);

    int const xMin = static_cast<int>((sMin * c) / d);
    int const xMax = static_cast<int>((sMax * c) / d);

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

void DrawSky(GameResources* res, SDL_Surface* display)
{
    auto& dir = res->dir;

    // Render sky
    // TODO: sky texture is being rendered flipped possibly.  Need to investigate.
    for (size_t i = 0; i < static_cast<size_t>(display->w); ++i)
    {
        // Treat sky as being projected in a cylinder, with deformation happening
        // around the edges of the screen.
        // The sky is wrapped around the cylinder 4 times (every 90 degrees the
        // image repeats)

        // Recall: left side of screen corresponds to +ve
        float worldPixelAngle = dir.theta + std::atan2((display->w / 2.0f) - i, dir.c);
        if (worldPixelAngle < 0.0f)
        {
            worldPixelAngle += 2 * M_PI_F;
        }
        else if (worldPixelAngle > 2 * M_PI_F)
        { // UGLY, but it works
            worldPixelAngle /= 2 * M_PI_F;
            worldPixelAngle -= std::floor(worldPixelAngle);
            worldPixelAngle *= 2 * M_PI_F;
        }

        // float const mirroredCoord = worldPixelAngle > 180.0f ? (1.0f -
        // (worldPixelAngle - 180.0f) / 180.0f) : (worldPixelAngle / 180.0f);

        float const finalCoord = (worldPixelAngle / (M_PI_F / 2)) - std::floor(worldPixelAngle / (M_PI_F / 2));

        DrawColumn(display, res->skyTranspose, i, 0, static_cast<size_t>(display->h / 2 + 1), finalCoord, 0, 1);
    }
}

void DrawFloor(GameResources* res, SDL_Surface* display)
{
    // Render floor
    for (int i = display->h / 2 + 1; i < display->h; ++i)
    {
        DrawSpan(display, res->floor, res->viewInverse, res->dir.c, 0, display->w - 1, i);
    }
}

void DrawFPS(GameResources* res, SDL_Surface* display, uint32_t const frameTime[2])
{
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
}

void DrawSprites(GameResources* res, SDL_Surface* display)
{

    // Render sprites

    auto& dir = res->dir;

    // Sort them
    // std::vector<std::pair<SDL_Surface*, Eigen::Vector2f>> sprites; // get from
    // entities

    AlignedVector<std::pair<SDL_Surface*, Eigen::Vector4f>> sorted;

    for (auto const& entity : res->entities)
    {
        Eigen::Vector4f const viewSpritePos = res->view * entity.pos.homogeneous();

        sorted.emplace_back(entity.texture, viewSpritePos);
        //
    }

    // Doomguy
    // Assume doomguy is facing [-1 0]
    Eigen::Vector2f doomGuyOrient{-1, 0};
    Eigen::Vector2f playerToDoomGuy = Eigen::Vector2f{550, 50} - Eigen::Vector2f{dir.pos(0), dir.pos(2)};
    auto const dot = playerToDoomGuy.dot(doomGuyOrient);
    auto const det = playerToDoomGuy(0) * doomGuyOrient(1) - playerToDoomGuy(1) * doomGuyOrient(0);
    auto angle = std::atan2(det, dot) + (M_PI_F / 8);

    /*if (angle < 0)
    {
        angle += 2 * M_PI_F;
    }*/

    int spriteSelect = static_cast<int>(4 + std::floor((angle) / (M_PI_F / 4)));
    if (spriteSelect >= 8)
    {
        spriteSelect = 0;
    }
    if (spriteSelect < 0)
    {
        spriteSelect = 7;
    }

    // TODO: figure out the actual transformation
    Eigen::Vector4f const doomSpriteViewPos = res->view * Eigen::Vector3f{550, 0, 50}.homogeneous();

    sorted.emplace_back(res->doomGuy[spriteSelect], doomSpriteViewPos);

    // Sort back to front
    std::sort(sorted.begin(), sorted.end(),
              [](std::pair<SDL_Surface const*, Eigen::Vector4f> const& x,
                 std::pair<SDL_Surface const*, Eigen::Vector4f> const& y) { return x.second(2) > y.second(2); });

    for (auto const& [texture, viewpos] : sorted)
    {
        RenderSprite(display, texture, res->dir.c, viewpos(0), viewpos(1), viewpos(2));
    }
}

// Eigen::Vector2f clipLinePlane(Eigen::Vector2f const& p1, Eigen::Vector2f const& p2, Eigen::Vector2f const& clip) {}

std::pair<float, float> clipLine(Eigen::Vector4f const& p1, Eigen::Vector4f const& p2, float const c,
                                 float const halfWidth)
{
    Eigen::Vector4f const pH = p2 - p1;
    Eigen::Vector2f const p = Eigen::Vector2f{pH(0), pH(2)};
    Eigen::Vector2f const p1_2d = Eigen::Vector2f{p1(0), p1(2)};
    Eigen::Vector2f const p2_2d = Eigen::Vector2f{p2(0), p2(2)};

    float p1c = 0;
    float p2c = 1;

    // Eigen::Vector2f const clipLeftP = Eigen::Vector2f{-halfWidth, c};
    Eigen::Vector2f const clipLeftN = Eigen::Vector2f{c, halfWidth};
    Eigen::Vector2f const clipRightN = {-c, halfWidth}; // points INWARDS

    float const clipNSqNm = clipLeftN.squaredNorm();

    float const clipLeftInverseP = clipLeftN.dot(p) / clipNSqNm;
    float const clipLeftInverseP1 = clipLeftN.dot(p1_2d) / clipNSqNm;

    float const clipRightInverseP2 = clipRightN.dot(p2_2d) / clipNSqNm;
    float const clipRightInverseP = clipRightN.dot(p) / clipNSqNm;

    if (clipLeftInverseP1 < 0)
    {
        float const diff = -clipLeftInverseP1;
        float const t = diff / clipLeftInverseP;
        p1c = t;
    }

    if (clipRightInverseP2 < 0)
    {
        float const diff = -clipRightInverseP2;
        float const t = diff / clipRightInverseP;
        p2c = 1 + t;
    }

    // STUDY: effect of constant height, interpolation between clipped coordinates relative to unclipped
    return {p1c, p2c};
}

void DrawWall(SDL_Surface const* texture, Eigen::Vector2f const& p1, Eigen::Vector2f const& p2, float const offset,
              float const height, float const c, Eigen::Matrix4f const& view, SDL_Surface* display)
{

    // Compute p1's view space coordinates

    Eigen::Vector4f const p1EyeBot = view * Eigen::Vector4f{p1(0), offset, p1(1), 1};
    Eigen::Vector4f const p1EyeTop = view * Eigen::Vector4f{p1(0), offset + height, p1(1), 1};
    Eigen::Vector4f const p2EyeBot = view * Eigen::Vector4f{p2(0), offset, p2(1), 1};
    Eigen::Vector4f const p2EyeTop = view * Eigen::Vector4f{p2(0), offset + height, p2(1), 1};

    float const displayWidthF = static_cast<float>(display->w);

    // Eigen::Vector4f const p1ViewXClamped = clipLineViewLeft(p1ViewTop, p2ViewTop, displayWidthF);
    auto const [p1ClipTop_t, p2ClipTop_t] = clipLine(p1EyeTop, p2EyeTop, c, displayWidthF / 2);
    auto const [p1ClipBot_t, p2ClipBot_t] = clipLine(p1EyeBot, p2EyeBot, c, displayWidthF / 2);

    if (p1ClipTop_t >= p2ClipTop_t)
    {
        return;
    }

    Eigen::Vector4f const p1ClipTop = p1EyeTop + p1ClipTop_t * (p2EyeTop - p1EyeTop);
    Eigen::Vector4f const p2ClipTop = p1EyeTop + p2ClipTop_t * (p2EyeTop - p1EyeTop);
    Eigen::Vector4f const p1ClipBot = p1EyeBot + p1ClipBot_t * (p2EyeBot - p1EyeBot);
    Eigen::Vector4f const p2ClipBot = p1EyeBot + p2ClipBot_t * (p2EyeBot - p1EyeBot);

    // Perspective division
    Eigen::Vector4f const p1ProjBot = c * p1ClipBot / p1ClipBot(2);
    Eigen::Vector4f const p1ProjTop = c * p1ClipTop / p1ClipTop(2);
    Eigen::Vector4f const p2ProjBot = c * p2ClipBot / p2ClipBot(2);
    Eigen::Vector4f const p2ProjTop = c * p2ClipTop / p2ClipTop(2);

    // Select texture based on right hand rule

    // SDL_Surface const* selectedTex = p1ProjBot(0) < p2ProjBot(0) ? frontTex : backTex;

    float const p1ProjTopClampedX =
        std::clamp(std::floor(p1ProjTop(0)), -displayWidthF / 2.0f, displayWidthF / 2.0f - 1);
    float const p2ProjTopClampedX =
        std::clamp(std::floor(p2ProjTop(0)), -displayWidthF / 2.0f, displayWidthF / 2.0f - 1);

    // float const p1ProjTopXClamped = std::clamp(p1ProjTop(0), -displayWidthF, displayWidthF);
    // float const p1ProjTopXClampDiff = p1ProjTopXClamped - p1ProjTop(0);
    // float const p2ProjTopXClamped = std::clamp(p2ProjTop(0), -displayWidthF, displayWidthF);

    // Select front or back texture depending on p1View.x and p2View.x -- right
    // hand rule

    float const topXDiff = p2ProjTopClampedX - p1ProjTopClampedX;
    // float const botXDiff = p2ProjBot(0) - p1ProjBot(0);

    float const topFrac = (p2ProjTop(1) - p1ProjTop(1)) / topXDiff;
    float const botFrac = (p2ProjBot(1) - p1ProjBot(1)) / topXDiff;
    // float const texFrac = 1 / topXDiff;

    ptrdiff_t const displayWidth = static_cast<ptrdiff_t>(display->w);

    // ptrdiff_t const startColumnUnclamped = static_cast<ptrdiff_t>(p1ProjTop(0)) + displayWidth / 2;
    // ptrdiff_t const endColumnUnclamped = static_cast<ptrdiff_t>(p2ProjTop(0)) + displayWidth / 2;

    // ptrdiff_t const startColumn = std::clamp(startColumnUnclamped, ptrdiff_t(0), displayWidth);
    // ptrdiff_t const endColumn = std::clamp(endColumnUnclamped, ptrdiff_t(0), displayWidth);

    float currProjTopY = p1ProjTop(1);

    // float currProjX = p1ProjTop(0) + p1ProjTopXClampDiff;

    float currTopY = display->h / 2 - currProjTopY;

    float currBotY = display->h / 2 - p1ProjBot(1);

    // float currX = float(startColumn - startColumnUnclamped) / topXDiff;

    Eigen::Vector4f const wallDirUnnormalized4 = p2EyeTop - p1EyeTop;
    Eigen::Vector2f const wallDirUnnormalized = Eigen::Vector2f{wallDirUnnormalized4(0), wallDirUnnormalized4(2)};
    float const wallDirSqNm = wallDirUnnormalized.squaredNorm();

    for (float it = p1ProjTopClampedX; it <= p2ProjTopClampedX; it++)
    {
        float const depth_i = c * p1ClipTop(1) / currProjTopY;
        float const disp_i = depth_i * it / c;

        Eigen::Vector2f const projPtUnnormalized =
            Eigen::Vector2f{disp_i, depth_i} - Eigen::Vector2f{p1ClipBot(0), p1ClipBot(2)};

        float const texT = p1ClipTop_t + wallDirUnnormalized.dot(projPtUnnormalized) / wallDirSqNm;

        ptrdiff_t const i = static_cast<ptrdiff_t>(it) + displayWidth / 2;

        DrawColumn(display, texture, i, static_cast<ptrdiff_t>(currTopY), static_cast<ptrdiff_t>(currBotY), texT, 0, 1);

        currTopY -= topFrac;
        currProjTopY += topFrac;
        currBotY -= botFrac;
        //++currProjX;
        // currX += texFrac;
    }

    return;
}

void DrawTwoSidedWall(SDL_Surface const* frontTex, SDL_Surface const* backTex, Eigen::Vector2f const& p1,
                      Eigen::Vector2f const& p2, float const offset, float const height, float const c,
                      Eigen::Matrix4f const& view, SDL_Surface* display)
{
    // Compute p1's view space coordinates

    // Eigen::Vector4f const p1EyeBot = view * Eigen::Vector4f{p1(0), offset, p1(1), 1};
    Eigen::Vector4f const p1EyeTop = view * Eigen::Vector4f{p1(0), offset + height, p1(1), 1};
    // Eigen::Vector4f const p2EyeBot = view * Eigen::Vector4f{p2(0), offset, p2(1), 1};
    Eigen::Vector4f const p2EyeTop = view * Eigen::Vector4f{p2(0), offset + height, p2(1), 1};

    Eigen::Vector4f const P = p2EyeTop - p1EyeTop;
    Eigen::Vector2f const N = {-P(2), P(0)};
    Eigen::Vector2f const P1 = {p1EyeTop(0), p1EyeTop(2)};

    float const whichSide = P1.dot(N) / N.squaredNorm();

    if (whichSide > 0)
    {
        DrawWall(frontTex, p1, p2, offset, height, c, view, display);
    }
    else
    {
        DrawWall(backTex, p2, p1, offset, height, c, view, display);
    }
}

void Draw(GameResources* res, uint32_t const frameTime[2])
{
    SDL_Surface* display = res->display;

    DrawSky(res, display);
    DrawFloor(res, display);
    DrawTwoSidedWall(res->wallFrontTranspose, res->wallBackTranspose, Eigen::Vector2f{-300, -300},
                     Eigen::Vector2f{300, -300}, 0, 100, res->dir.c, res->view, display);
    DrawTwoSidedWall(res->wallFrontTranspose, res->wallBackTranspose, Eigen::Vector2f{300, -300},
                     Eigen::Vector2f{700, -100}, 0, 150, res->dir.c, res->view, display);
    DrawTwoSidedWall(res->wallFrontTranspose, res->wallBackTranspose, Eigen::Vector2f{700, -100},
                     Eigen::Vector2f{700, 400}, 0, 150, res->dir.c, res->view, display);
    DrawTwoSidedWall(res->wallFrontTranspose, res->wallBackTranspose, Eigen::Vector2f{700, 400},
                     Eigen::Vector2f{300, 600}, 0, 150, res->dir.c, res->view, display);
    DrawTwoSidedWall(res->wallFrontTranspose, res->wallBackTranspose, Eigen::Vector2f{300, 600},
                     Eigen::Vector2f{-300, 600}, 0, 100, res->dir.c, res->view, display);
    DrawSprites(res, display);
    DrawFPS(res, display, frameTime);
}

void GameLoop(void* const arg)
{
    GameResources* const res = static_cast<GameResources* const>(arg);

    static uint32_t prevFrameTime[2] = {};

    prevFrameTime[0] = prevFrameTime[1];
    prevFrameTime[1] = SDL_GetTicks();

    if (ProcessEvent(res->dir, res->move, res->windowID) == 0)
    {
        // FreeGameResources(res); //browser specific: it handles cleanup
        emscripten_cancel_main_loop();
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Exiting!\n");
        SDL_Quit();
        std::exit(0); // Reason for not plain returning: emscripten won't call
                      // global destructors when main or the main loop exits.
    }
    UpdateViewMatrices(res->dir, res->viewInverse, res->view);
    MovePlayer(res->dir, res->viewInverse, res->move, prevFrameTime[1] - prevFrameTime[0]);

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

int main(int, char* [])
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
    res.window = SDL_CreateWindow("Floor - SDL2 version", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480,
                                  SDL_WINDOW_SHOWN);

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

    int xres{};
    int yres{};
    if (SDL_GetRendererOutputSize(res.renderer, &xres, &yres) < 0)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not get renderer output size: %s\n", SDL_GetError());
        return 1;
    }

    res.display = SDL_CreateRGBSurfaceWithFormat(0, xres, yres, 32, SDL_PIXELFORMAT_RGBA8888);
    if (res.display == nullptr)
    {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "could not create surface: %s\n", SDL_GetError());
        return 1;
    }

    res.texture = SDL_CreateTexture(res.renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, res.display->w,
                                    res.display->h);

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
    res.wallFrontTranspose = LoadTexture("../assets/wallFront.bmp", res.display->format);
    res.wallBackTranspose = LoadTexture("../assets/wallBack.bmp", res.display->format);
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

    // Objects
    res.entities.emplace_back("column1", res.columnSprite, Eigen::Vector3f{500, 0, 0});
    res.entities.emplace_back("vial", res.vialSprite, Eigen::Vector3f{500, 25, 100});
    res.entities.emplace_back("health", res.healthSprite, Eigen::Vector3f{500, 0, 200});
    res.entities.emplace_back("column2", res.columnSprite, Eigen::Vector3f{500, 0, 300});

    emscripten_set_main_loop_arg(GameLoop, &res, 0, 1);

    return 0;
}

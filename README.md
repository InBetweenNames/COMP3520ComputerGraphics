UWindsor COMP-3520 SDL2 Project Template (CMake version)
===

Floordemo branch
---

This branch contains source code for some 2.5D rendering functions written
for software rendering.  The code needs a bit of cleaning up, but otherwise
should work fine.  Pull requests are welcome to improve the code.

The code is written in mostly C such that students without a C++ background
can more easily understand it.  There are some parts that need to be overhauled,
like the conversion from floating point coordinates to integer coordinates.
However, the basic ideas should be evident.  In class, we'll discuss how
this would have been implemented using fixed-point arithmetic.

A live demo compiled using Emscripten and WebAssembly is available [here](https://inbetweennames.github.io/SDL2TemplateCMake/)

Controls:
---

Mouse - look left/right
W, A, S, D - move forwards, backwards, upwards, downwards
T, G -- raise and lower height
Y, H -- adjust focal distance (distance to screen plane)

Template
---

This template is intended for students in the COMP-3520 Introduction to Computer Graphics course
at the University of Windsor, however it should serve as a useful template for anyone interested in
getting started with the [SDL2](http://libsdl.org/) library quickly on non-Windows platforms.
In contrast to the [Visual Studio template](https://github.com/InBetweenNames/SDL2Template), this version
uses dynamic linking by default and uses your installed system libraries.

The following dependencies are required:
* [CMake](https://cmake.org/)
* [SDL2](http://libsdl.org/) 
* [SDL2_ttf](https://www.libsdl.org/projects/SDL_ttf/) library for easy text rendering
* [freetype2](https://www.freetype.org/) which `SDL2_ttf` uses for the actual work

It's very easy to get started with SDL2 on most linux distributions.  Just ensure you have the above packages installed along with their header files (usually requires a `-dev` package)

On Debian based systems, the packages will probably be called `libsdl2-dev` and `libsdl2-ttf-dev`.
On Arch Linux, look for the packages `sdl2` and `sdl2_ttf`.

Mac users should be able to use Homebrew or a similar package manager to install the needed dependencies.
If you need help, just send me an email.

Setup
---

Once you've installed these packages, you'll be all set to start your first assignment.
Clone this repository somewhere using Git (in a shell):

~~~
git clone https://github.com/InBetweenNames/SDL2TemplateCMake.git
cd SDL2TemplateCMake
~~~

Next, enter the build directory:

~~~
cd build
~~~

Run CMake:

~~~
cmake ..
~~~

If this runs without errors, you're ready to build:

~~~
cmake --build .
~~~

Now, run the demo:

~~~
./main
~~~

Note that `iosevka-regular.ttf` must be in the working directory of `main` for it to work.
In practice, this means you need to be in the `build` directory when running `main`.
I would welcome a pull request that removes this restriction.

Setup for Emscripten
---

This template now supports building using [Emscripten](https://kripken.github.io/emscripten-site/) for compiling your SDL2-based
C or C++ code directly to your web browser using WebAssembly.  To use it, ensure you have the [Emscripten SDK](https://github.com/emscripten-core/emsdk)
installed and in your PATH (use `emsdk install` and `emsdk activate`, following all instructions), and then:

~~~
cd build-wasm
./build_with_emscripten.sh
~~~

This will configure, compile, link, and run your project directly in your web browser.

The demo
---

The demo code will change periodically to help you with your newest assignments.
You can clone this project as many times as you need for different assignments.

Recommended practices
---

Students who know C++ are encouraged to use it, however, C++ is not a requirement for the course.
The sample code provided is mostly C compatible for the benefit of students who haven't had much C++ exposure yet.

When we get to the more mathy parts of the course, if you have a good handle on C++, consider using
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) for your Linear Algebra needs.

Extra goodies:
---

Although this template has everything you need to succeed in the course, in your own personal projects
it's likely you'll want to go even further.  Consider adding the following libraries for your arsenal:

* [SDL2_image](https://www.libsdl.org/projects/SDL_image/) for easy image loading from a variety of formats
* [SDL2_net](https://www.libsdl.org/projects/SDL_net/) for a basic cross-platform networking library
* [SDL2_mixer](https://www.libsdl.org/projects/SDL_mixer/) for sound rendering
* [SDL2_rtf](https://www.libsdl.org/projects/SDL_rtf/) for basic document handling (RTF)

Bugs:
---

If you find any problems with the template, please let me know by either creating an Issue on the project page or sending
me an email.

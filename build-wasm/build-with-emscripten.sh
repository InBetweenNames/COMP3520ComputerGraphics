#!/bin/bash


if [[ -z $(which emcmake) ]]; then
  echo "No emcmake detected.  Is Emscripten installed and in your PATH?"
  echo "Remember to follow the Emscripten SDK instructions."
  exit 1
fi

emcmake cmake .. && cmake --build . && emrun main.html

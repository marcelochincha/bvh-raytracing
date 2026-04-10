# SR-LEC

A lightweight 3D software renderer built with SDL, designed for educational purposes and real-time visualization.

## Setup

You mush and your own SDL2 library binaries to build and run the project. You can download SDL2 from the official website: https://github.com/libsdl-org/SDL/releases/

Directory structure:
- lib/                    - SDL2 library files 
- third_party/SDL2/       - SDL2 header files (.h definitions)
- bin/                    - Compiled executable files (you should add dynamically linked SDL2 library files (SDL2.dll for Windows, SDL2.so for Linux) here as well)
- src/                    - Source code for the software renderer
- res/                    - Resource files (textures, models, etc.)
- tests/                  - High level tests for physics or algorithms prototypes
- Makefile                - Build configuration file (Configure this as needed for your platform and SDL2 setup (default is for Windows with SDL2.dll in bin/))

You can add libraries and other headers in lib and third_party as needed, but the core renderer and logic code should go in src/.

## Features

- **3D Rendering**: Software-based 3D graphics pipeline
- **SDL Integration**: Cross-platform graphics and input handling
- **Lightweight**: Minimal dependencies for easy setup and learning
- **Real-time**: Optimized for interactive applications
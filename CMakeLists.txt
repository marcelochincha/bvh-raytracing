cmake_minimum_required(VERSION 3.16)
project(bvh_raytracer LANGUAGES CXX)

# ---- Language & build settings ----------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---- SDL2 (the only external dependency) -------------------------------------
# Works on macOS (brew install sdl2), Windows (vcpkg / system) and Linux.
find_package(SDL2 REQUIRED)

# ---- Sources: every .cpp under src/, recursively ----------------------------
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} WIN32 ${APP_SOURCES})

# Headers include each other as <math/...>, <renderer/...>, <game/...>, so the
# include root is src/ itself.
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/src")

# ---- Link SDL2 (handle both modern imported targets and legacy variables) ----
if(TARGET SDL2::SDL2)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2)
    if(TARGET SDL2::SDL2main)
        target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2main)
    endif()
else()
    target_include_directories(${PROJECT_NAME} PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${SDL2_LIBRARIES})
endif()

# ---- Intel Embree (optional, external reference for the study) ----------------
# When present it is linked in and WITH_EMBREE is defined so the benchmark adds
# an "Embree" row. When absent the project still builds; the benchmark just skips
# that row. (macOS: `brew install embree`.)
option(WITH_EMBREE "Use Intel Embree as an external reference" ON)
if(WITH_EMBREE)
    find_package(embree 4 QUIET)
    if(TARGET embree)
        message(STATUS "Embree ${embree_VERSION} found -> WITH_EMBREE enabled")
        target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_EMBREE)
        target_link_libraries(${PROJECT_NAME} PRIVATE embree)
    else()
        message(STATUS "Embree NOT found -> building without it (benchmark will skip the Embree row)")
    endif()
endif()

# ---- Tune for the host CPU (NEON on Apple Silicon, AVX on x86) ----------------
target_compile_options(${PROJECT_NAME} PRIVATE -march=native)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W3 /utf-8)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# ---- Where the binary lands & where it reads res/ from -----------------------
# Default output to bin/ next to the source tree so res/ (textures) is found.
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

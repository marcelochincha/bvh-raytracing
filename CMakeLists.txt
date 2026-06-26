cmake_minimum_required(VERSION 3.16)
project(bvh_raytracer LANGUAGES CXX)

# ---- Language & build settings ----------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---- SDL2 (bundled: third_party/SDL2/ + lib/) --------------------------------
# On macOS a system/Homebrew install is tried first via find_package.
# On Windows the bundled static libs in lib/ are used directly.
find_package(SDL2 QUIET)

if(NOT TARGET SDL2::SDL2)
    find_path(SDL2_INCLUDE_DIR SDL.h
        PATHS "${CMAKE_SOURCE_DIR}/third_party/SDL2"
        NO_DEFAULT_PATH)

    find_library(SDL2_LIB NAMES SDL2 SDL2dll
        PATHS "${CMAKE_SOURCE_DIR}/lib"
        NO_DEFAULT_PATH)

    find_library(SDL2MAIN_LIB NAMES SDL2main
        PATHS "${CMAKE_SOURCE_DIR}/lib"
        NO_DEFAULT_PATH)

    if(NOT SDL2_INCLUDE_DIR OR NOT SDL2_LIB)
        message(FATAL_ERROR
            "SDL2 not found in third_party/SDL2/ or lib/.\n"
            "Make sure the bundled SDL2 headers and libs are present in the repo.")
    endif()

    message(STATUS "SDL2 (bundled): ${SDL2_INCLUDE_DIR}")

    get_filename_component(SDL2_INCLUDE_PARENT "${SDL2_INCLUDE_DIR}" DIRECTORY)

    add_library(SDL2::SDL2 IMPORTED STATIC GLOBAL)
    set_target_properties(SDL2::SDL2 PROPERTIES
        IMPORTED_LOCATION "${SDL2_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${SDL2_INCLUDE_PARENT}")

    if(SDL2MAIN_LIB)
        add_library(SDL2::SDL2main IMPORTED STATIC GLOBAL)
        set_target_properties(SDL2::SDL2main PROPERTIES
            IMPORTED_LOCATION "${SDL2MAIN_LIB}")
    endif()
endif()

# ---- Sources: every .cpp under src/, recursively ----------------------------
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} WIN32 ${APP_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/src")

# ---- Link SDL2 --------------------------------------------------------------
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2)
if(TARGET SDL2::SDL2main)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2main)
endif()
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE mingw32 psapi)
endif()

# ---- OpenCL GPU backend (optional) ------------------------------------------
# Windows: bundled lib/libOpenCL.a + third_party/CL/ headers.
# macOS:   system -framework OpenCL (no lib needed).
# Enable:  cmake -DWITH_OPENCL=ON ..
option(WITH_OPENCL "Enable OpenCL GPU ray-tracing backend [G key]" OFF)
if(WITH_OPENCL)
    if(APPLE)
        message(STATUS "OpenCL (macOS framework) -> WITH_OPENCL enabled")
        target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_OPENCL)
        target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/third_party")
        target_link_libraries(${PROJECT_NAME} PRIVATE "-framework OpenCL")
    else()
        find_library(OCL_LIB NAMES OpenCL
            PATHS "${CMAKE_SOURCE_DIR}/lib"
            NO_DEFAULT_PATH)
        if(NOT OCL_LIB)
            find_library(OCL_LIB NAMES OpenCL)
        endif()
        if(OCL_LIB)
            message(STATUS "OpenCL (bundled) -> WITH_OPENCL enabled (${OCL_LIB})")
            target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_OPENCL)
            target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/third_party")
            target_link_libraries(${PROJECT_NAME} PRIVATE "${OCL_LIB}")
        else()
            message(WARNING "libOpenCL.a not found in lib/ -> WITH_OPENCL disabled")
        endif()
    endif()
endif()

# ---- Intel Embree (bundled: third_party/embree4/ + lib/) --------------------
option(WITH_EMBREE "Use bundled Intel Embree 4" ON)
if(WITH_EMBREE)
    set(EMBREE_INCLUDE "${CMAKE_SOURCE_DIR}/third_party")
    set(EMBREE_LIB     "${CMAKE_SOURCE_DIR}/lib/embree4.lib")

    if(EXISTS "${EMBREE_INCLUDE}/embree4/rtcore.h" AND EXISTS "${EMBREE_LIB}")
        message(STATUS "Embree 4 (bundled) -> WITH_EMBREE enabled")
        add_library(embree4::embree4 IMPORTED STATIC GLOBAL)
        set_target_properties(embree4::embree4 PROPERTIES
            IMPORTED_LOCATION "${EMBREE_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${EMBREE_INCLUDE}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_EMBREE)
        target_link_libraries(${PROJECT_NAME} PRIVATE embree4::embree4)
    else()
        message(STATUS "Embree NOT found in third_party/embree4/ -> building without it")
    endif()
endif()

# ---- Tune for the host CPU --------------------------------------------------
target_compile_options(${PROJECT_NAME} PRIVATE -march=native)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W3 /utf-8)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# ---- Where the binary lands -------------------------------------------------
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# ---- Copy runtime DLLs next to the exe (Windows) ----------------------------
if(WIN32)
    foreach(DLL SDL2.dll libwinpthread-1.dll)
        if(EXISTS "${CMAKE_SOURCE_DIR}/bin/${DLL}")
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${CMAKE_SOURCE_DIR}/bin/${DLL}"
                    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/${DLL}"
                COMMENT "Copying ${DLL}")
        endif()
    endforeach()

    foreach(DLL embree4.dll tbb12.dll tbbmalloc.dll)
        if(EXISTS "${CMAKE_SOURCE_DIR}/bin/${DLL}")
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${CMAKE_SOURCE_DIR}/bin/${DLL}"
                    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/${DLL}"
                COMMENT "Copying ${DLL}")
        endif()
    endforeach()
endif()

# ---- run target: cmake --build . --target run --------------------------------
add_custom_target(run
    COMMAND ${PROJECT_NAME}
    DEPENDS ${PROJECT_NAME}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Running ${PROJECT_NAME}..."
)

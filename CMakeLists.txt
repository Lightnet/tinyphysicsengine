cmake_minimum_required(VERSION 3.15)
project(tinyphysicsengine LANGUAGES C CXX)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(FetchContent)
#================================================
# Fetch raylib
#================================================
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
    USES_TERMINAL_DOWNLOAD TRUE
)
FetchContent_MakeAvailable(raylib)
#================================================
# raygui
#================================================
FetchContent_Declare(
    raygui
    GIT_REPOSITORY https://github.com/raysan5/raygui.git
    GIT_TAG 4.0
    GIT_SHALLOW TRUE
    USES_TERMINAL_DOWNLOAD TRUE
)
FetchContent_MakeAvailable(raygui)

FetchContent_Declare(
    SDL2
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-2.32.10
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(SDL2)


# set(EXPORT_RAYLIB_APP ON)
set(EXPORT_RAYLIB_APP OFF)
if(${EXPORT_RAYLIB_APP})
    set(APP_NAME demo)
    set(SRC_FILES
        #helper for header and c
    )
    add_executable(${APP_NAME}
        ${SRC_FILES}
        # examples/raylib_window.c 
        examples/raylib/raylib_tpe.c 
    )
    target_link_libraries(${APP_NAME} PRIVATE 
        raylib
    )
    target_include_directories(${APP_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # ${CMAKE_CURRENT_SOURCE_DIR}
    )
    if(WIN32)
        # ws2_32 # network
        target_link_libraries(${APP_NAME} PRIVATE ws2_32 gdi32 user32 shell32)
        target_link_options(${APP_NAME} PRIVATE
            -static-libgcc
            -static-libstdc++  # Uncomment if C++ code is used
            -static           # Avoid full static linking to prevent issues with system libraries
        )
    endif()
endif(${EXPORT_RAYLIB_APP})




set(EXPORT_SDL2_APP ON)
# set(EXPORT_SDL2_APP OFF)
if(${EXPORT_SDL2_APP})
    set(APP_SDL2_NAME sdl2_demo)
    set(SRC_FILES
        #helper for header and c
        programs/helper.h
    )
    add_executable(${APP_SDL2_NAME}
        ${SRC_FILES}
        # examples/sdl2_demo.c 
        programs/2d.c
    )
    target_link_libraries(${APP_SDL2_NAME} PRIVATE 
        SDL2::SDL2main 
        SDL2::SDL2
    )
    target_include_directories(${APP_SDL2_NAME} PUBLIC
        ${SDL2_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # ${CMAKE_CURRENT_SOURCE_DIR}
    )
    if(WIN32)
        # ws2_32 # network
        target_link_libraries(${APP_SDL2_NAME} PRIVATE ws2_32 gdi32 user32 shell32)
        target_link_options(${APP_SDL2_NAME} PRIVATE
            -static-libgcc
            -static-libstdc++  # Uncomment if C++ code is used
            -static           # Avoid full static linking to prevent issues with system libraries
        )
    endif()
endif(${EXPORT_SDL2_APP})
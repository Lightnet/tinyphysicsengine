cmake_minimum_required(VERSION 3.15)
project(tinyphysicsengine LANGUAGES C CXX)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# set(EXAMPLE_TESTS ON)
set(EXAMPLE_TESTS OFF)

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



if(EXAMPLE_TESTS)
    FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-2.32.10
        GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(SDL2)
endif(EXAMPLE_TESTS)


set(EXPORT_RAYLIB_APP ON)
# set(EXPORT_RAYLIB_APP OFF)
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


if(EXAMPLE_TESTS)
    set(EXAMPLES_FILES
        programs/2d.c
        programs/car.c
        programs/cubes.c
        programs/envaccel.c
        programs/heightmap.c
        programs/hello.c
        programs/hello2.c
        programs/main.c
        programs/player.c
        programs/shoot.c
        programs/stack.c
        programs/test.c
        programs/testGeneral.c
        programs/water.c
    )
    set(EXPORT_SDL2_APP ON)
    # set(EXPORT_SDL2_APP OFF)
    set(APP_SDL2_NAME_BASE "sdl2_") # prefix for the binary names

    if(EXPORT_SDL2_APP)
        
        foreach(EXAMPLE_SRC ${EXAMPLES_FILES})
            # ---- derive a nice target name from the file name ----
            get_filename_component(EXAMPLE_NAME ${EXAMPLE_SRC} NAME_WE)   # strip .c
            set(TARGET_NAME "${APP_SDL2_NAME_BASE}${EXAMPLE_NAME}")

            # ---- create the executable ----
            add_executable(${TARGET_NAME}
                ${EXAMPLE_SRC}
                programs/helper.h          # shared helper header (optional)
            )

            # ---- link SDL2 (adjust if you use find_package(SDL2) instead) ----
            target_link_libraries(${TARGET_NAME} PRIVATE
                SDL2::SDL2main
                SDL2::SDL2
            )

            # ---- include directories (common for all examples) ----
            target_include_directories(${TARGET_NAME} PUBLIC
                ${SDL2_SOURCE_DIR}/include
                ${CMAKE_CURRENT_SOURCE_DIR}/include
            )

            # ---- Windows-specific static linking (optional) ----
            if(WIN32)
                target_link_libraries(${TARGET_NAME} PRIVATE ws2_32 gdi32 user32 shell32)
                target_link_options(${TARGET_NAME} PRIVATE
                    -static-libgcc
                    -static-libstdc++   # only if you have C++ sources
                    -static             # careful: not fully static, avoids system-lib issues
                )
            endif()
        endforeach()
    endif(EXPORT_SDL2_APP)


endif(EXAMPLE_TESTS)


# if(EXAMPLE_TESTS)
#     set(APP_SDL2_1 demo_car)
#     add_executable(${APP_SDL2_1}
#         ESPboy_car/ESPboy_car.c
#     )
#     target_link_libraries(${APP_SDL2_1} PRIVATE 
#         raylib
#     )
#     target_include_directories(${APP_SDL2_1} PUBLIC
#         ${CMAKE_CURRENT_SOURCE_DIR}/ESPboy_car
#         # ${CMAKE_CURRENT_SOURCE_DIR}
#     )
#     if(WIN32)
#         # ws2_32 # network
#         target_link_libraries(${APP_SDL2_1} PRIVATE ws2_32 gdi32 user32 shell32)
#         target_link_options(${APP_SDL2_1} PRIVATE
#             -static-libgcc
#             -static-libstdc++  # Uncomment if C++ code is used
#             -static           # Avoid full static linking to prevent issues with system libraries
#         )
#     endif()
# endif(EXAMPLE_TESTS)
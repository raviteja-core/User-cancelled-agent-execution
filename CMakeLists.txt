cmake_minimum_required(VERSION 3.20)
project(PaperCutter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Werror -pedantic)
endif()

add_subdirectory(core)
add_subdirectory(controller_app)

enable_testing()
add_subdirectory(tests/cpp)

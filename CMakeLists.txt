cmake_minimum_required(VERSION 3.20)
# Workaround: some FetchContent deps declare cmake_minimum_required < 3.5
if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "")
project(fsae_lapsim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml-cpp raylib googletest)

# --- Static library: lapsim_core ---
file(GLOB_RECURSE LAPSIM_CORE_SRC CONFIGURE_DEPENDS "src/*.cpp")
add_library(lapsim_core STATIC ${LAPSIM_CORE_SRC})
target_include_directories(lapsim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(lapsim_core PUBLIC yaml-cpp::yaml-cpp raylib)

# --- CLI executable ---
add_executable(lapsim_cli apps/lapsim_cli.cpp)
target_link_libraries(lapsim_cli PRIVATE lapsim_core)

# --- Visualizer executable ---
add_executable(lapsim_viz apps/lapsim_viz.cpp)
target_link_libraries(lapsim_viz PRIVATE lapsim_core)

# --- Tests ---
enable_testing()
file(GLOB_RECURSE LAPSIM_TEST_SRC CONFIGURE_DEPENDS "tests/*.cpp")
add_executable(lapsim_tests ${LAPSIM_TEST_SRC})
target_link_libraries(lapsim_tests PRIVATE lapsim_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(lapsim_tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

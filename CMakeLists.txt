cmake_minimum_required(VERSION 3.16)
project(Golem2000 VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ──────────────────────────────────────────────
# Dependencies
# ──────────────────────────────────────────────

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(yaml-cpp REQUIRED)

find_package(raylib 4.0 REQUIRED)

# ──────────────────────────────────────────────
# Core library
# ──────────────────────────────────────────────

add_library(golem_core STATIC
    src/Robot.cpp
    src/SceneRenderer.cpp
)
target_include_directories(golem_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(golem_core PUBLIC Eigen3::Eigen yaml-cpp raylib)

# ──────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────

enable_testing()
add_subdirectory(tests)

# ──────────────────────────────────────────────
# Tools
# ──────────────────────────────────────────────

add_executable(validate_robot tools/validate_robot.cpp)
target_link_libraries(validate_robot PRIVATE golem_core)

add_executable(visualize_robot tools/visualize_robot.cpp)
target_link_libraries(visualize_robot PRIVATE golem_core)

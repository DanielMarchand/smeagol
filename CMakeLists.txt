cmake_minimum_required(VERSION 3.16)
project(Smeagol VERSION 0.1.0 LANGUAGES CXX)

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

add_library(smeagol_core STATIC
    src/Robot.cpp
    src/SceneRenderer.cpp
    src/SnapshotRenderer.cpp
    src/VideoRenderer.cpp
    src/Simulator.cpp
    src/FitnessEvaluator.cpp
    src/RobotFactory.cpp
)
target_include_directories(smeagol_core PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(smeagol_core PUBLIC Eigen3::Eigen yaml-cpp raylib)

# ──────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────

enable_testing()
add_subdirectory(tests)

# ──────────────────────────────────────────────
# Tools
# ──────────────────────────────────────────────

add_executable(validate_robot tools/validate_robot.cpp)
target_link_libraries(validate_robot PRIVATE smeagol_core)

add_executable(visualize_robot tools/visualize_robot.cpp)
target_link_libraries(visualize_robot PRIVATE smeagol_core)

add_executable(snapshot_robot tools/snapshot_robot.cpp)
target_link_libraries(snapshot_robot PRIVATE smeagol_core)

add_executable(deformation_series tools/deformation_series.cpp)
target_link_libraries(deformation_series PRIVATE smeagol_core)

add_executable(run_simulation tools/run_simulation.cpp)
target_link_libraries(run_simulation PRIVATE smeagol_core)

add_executable(evaluate_fitness tools/evaluate_fitness.cpp)
target_link_libraries(evaluate_fitness PRIVATE smeagol_core)

add_executable(random_robots tools/random_robots.cpp)
target_link_libraries(random_robots PRIVATE smeagol_core)

add_executable(generate_robots tools/generate_robots.cpp)
target_link_libraries(generate_robots PRIVATE smeagol_core)

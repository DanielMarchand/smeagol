add_executable(test_dependencies test_dependencies.cpp)
target_link_libraries(test_dependencies PRIVATE Eigen3::Eigen yaml-cpp raylib)
add_test(NAME DependenciesInstalled COMMAND test_dependencies)

add_executable(test_robot_parts test_robot_parts.cpp)
target_link_libraries(test_robot_parts PRIVATE golem_core)
add_test(NAME RobotParts COMMAND test_robot_parts)

add_executable(test_robot test_robot.cpp)
target_link_libraries(test_robot PRIVATE golem_core)
add_test(NAME Robot COMMAND test_robot)

add_executable(test_yaml test_yaml.cpp)
target_link_libraries(test_yaml PRIVATE golem_core)
add_test(NAME YAMLRoundtrip COMMAND test_yaml)

add_executable(test_scene_renderer test_scene_renderer.cpp)
target_link_libraries(test_scene_renderer PRIVATE golem_core)
add_test(NAME SceneRenderer COMMAND test_scene_renderer)

add_executable(test_snapshot test_snapshot.cpp)
target_link_libraries(test_snapshot PRIVATE golem_core)
add_test(NAME SnapshotRenderer COMMAND test_snapshot)

add_executable(test_debug_actuator test_debug_actuator.cpp)
target_link_libraries(test_debug_actuator PRIVATE golem_core)
add_test(NAME DebugActuator COMMAND test_debug_actuator)

add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator PRIVATE golem_core)
add_test(NAME Simulator COMMAND test_simulator)

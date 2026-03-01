add_executable(test_dependencies test_dependencies.cpp)
target_link_libraries(test_dependencies PRIVATE Eigen3::Eigen yaml-cpp raylib)
add_test(NAME DependenciesInstalled COMMAND test_dependencies)

add_executable(test_robot_parts test_robot_parts.cpp)
target_link_libraries(test_robot_parts PRIVATE smeagol_core)
add_test(NAME RobotParts COMMAND test_robot_parts)

add_executable(test_robot test_robot.cpp)
target_link_libraries(test_robot PRIVATE smeagol_core)
add_test(NAME Robot COMMAND test_robot)

add_executable(test_yaml test_yaml.cpp)
target_link_libraries(test_yaml PRIVATE smeagol_core)
add_test(NAME YAMLRoundtrip COMMAND test_yaml)

add_executable(test_scene_renderer test_scene_renderer.cpp)
target_link_libraries(test_scene_renderer PRIVATE smeagol_core)
add_test(NAME SceneRenderer COMMAND test_scene_renderer)

add_executable(test_snapshot test_snapshot.cpp)
target_link_libraries(test_snapshot PRIVATE smeagol_core)
add_test(NAME SnapshotRenderer COMMAND test_snapshot)

add_executable(test_debug_actuator test_debug_actuator.cpp)
target_link_libraries(test_debug_actuator PRIVATE smeagol_core)
add_test(NAME DebugActuator COMMAND test_debug_actuator)

add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator PRIVATE smeagol_core)
add_test(NAME Simulator COMMAND test_simulator)

add_executable(test_neural_actuator test_neural_actuator.cpp)
target_link_libraries(test_neural_actuator PRIVATE smeagol_core)
add_test(NAME NeuralActuator COMMAND test_neural_actuator)

add_executable(test_fitness test_fitness.cpp)
target_link_libraries(test_fitness PRIVATE smeagol_core)
add_test(NAME Fitness COMMAND test_fitness)

add_executable(test_video_renderer test_video_renderer.cpp)
target_link_libraries(test_video_renderer PRIVATE smeagol_core)
add_test(NAME VideoRenderer COMMAND test_video_renderer)

add_executable(test_robot_factory test_robot_factory.cpp)
target_link_libraries(test_robot_factory PRIVATE smeagol_core)
add_test(NAME RobotFactory COMMAND test_robot_factory)

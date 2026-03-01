add_executable(test_dependencies test_dependencies.cpp)
target_link_libraries(test_dependencies PRIVATE Eigen3::Eigen yaml-cpp raylib)
add_test(NAME DependenciesInstalled COMMAND test_dependencies)

add_executable(test_robot_parts test_robot_parts.cpp)
target_link_libraries(test_robot_parts PRIVATE golem_core)
add_test(NAME RobotParts COMMAND test_robot_parts)

add_executable(test_robot test_robot.cpp)
target_link_libraries(test_robot PRIVATE golem_core)
add_test(NAME Robot COMMAND test_robot)

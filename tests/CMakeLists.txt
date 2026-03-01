add_executable(test_dependencies test_dependencies.cpp)

target_link_libraries(test_dependencies
    PRIVATE
        Eigen3::Eigen
        yaml-cpp
        raylib
)

add_test(NAME DependenciesInstalled COMMAND test_dependencies)

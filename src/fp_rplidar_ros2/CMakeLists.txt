# Set the minimum required version for CMake.
# Version 3.8 is the minimum for ROS 2.
cmake_minimum_required(VERSION 3.8)

# Define the project name. This should match the package name in your package.xml.
project(fp_rplidar_ros2)

# Set the C++ standard to C++17.
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# Find all required dependencies. This should always be done before creating targets.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Use find_library to locate the RPLIDAR SDK library by its file name.
# This is a more direct way to find a specific library file.
find_library(RPLIDAR_SDK_LIBRARY
        NAMES sl_lidar_sdk
        HINTS
        ../rplidar_sdk/output/Linux/Release
)

# Add the executable for your node.
add_executable(fp_rplidar_publisher src/fp_rplidar_publisher.cpp)

# Define the dependencies for the executable using ament_target_dependencies.
ament_target_dependencies(fp_rplidar_publisher rclcpp sensor_msgs)

# Include directories for headers.
# We add the package's `include` folder and the SDK's include directories.
# The `PUBLIC` keyword means downstream packages will also inherit this include path.
target_include_directories(fp_rplidar_publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        ../rplidar_sdk/sdk/include
)

# Link your executable against the RPLIDAR SDK library.
# We use the variable created by `find_library`.
target_link_libraries(fp_rplidar_publisher ${RPLIDAR_SDK_LIBRARY})

# Install the executable so that it is available after building.
# This is the `install` target that fixes the original colcon warning.
install(TARGETS
        fp_rplidar_publisher
        DESTINATION lib/${PROJECT_NAME}
)

# Install the header files. This is important if other packages will use your code.
install(DIRECTORY
        include/
        DESTINATION include/${PROJECT_NAME}
)

# Call ament_package() at the very end to finalize the package configuration.
ament_package()

# You can add the compiler flags after everything else.
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

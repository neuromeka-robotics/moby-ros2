cmake_minimum_required(VERSION 3.5)
project(moby_bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RailSensor.msg"
  "msg/CameraAngle.msg"
  "msg/CameraHeight.msg"
  "msg/OdomRatio.msg"
#  DEPENDENCIES geometry_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# install(DIRECTORY
#   src
#   DESTINATION share/${PROJECT_NAME}/
# )

install(DIRECTORY
  indy_utils moby_utils
  DESTINATION lib/python3.10/site-packages
)

install(DIRECTORY
  launch controller param rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  ${PROJECT_NAME}/indy_driver.py   
  ${PROJECT_NAME}/moby_driver.py
  ${PROJECT_NAME}/controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

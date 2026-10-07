cmake_minimum_required(VERSION 3.8)
project(trashcan_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Runs the whole simulation headless. A private Gazebo partition keeps it
  # from talking to a simulation already running on the same machine.
  add_launch_test(test/test_sim.launch.py
    TIMEOUT 300
    ENV GZ_PARTITION=trashcan_bringup_test ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
endif()

ament_package()

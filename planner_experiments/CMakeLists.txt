cmake_minimum_required(VERSION 3.0.2)
project(planner_experiments)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  isweep_planner
  map_server
  nav_msgs
  roscpp
  roslib
  std_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planner_experiments_core
  CATKIN_DEPENDS geometry_msgs isweep_planner map_server nav_msgs roscpp roslib std_msgs visualization_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../isweep_planner/include/isweep_planner/third_party
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(planner_experiments_core STATIC
  src/experiment_types.cpp
  src/planner_adapter.cpp
  src/trajectory_validator.cpp
  src/geometry_consistency_checker.cpp
  src/metrics_collector.cpp
  src/result_writer.cpp
  src/experiment_runner.cpp
)
target_compile_definitions(planner_experiments_core PUBLIC EIGEN_MAX_ALIGN_BYTES=0)
target_compile_definitions(planner_experiments_core PRIVATE
  PLANNER_EXPERIMENTS_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
add_dependencies(planner_experiments_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_experiments_core
  isweep_planner_core
  ${catkin_LIBRARIES}
  OpenSSL::Crypto
  yaml-cpp
)

add_executable(experiment_runner_node src/experiment_runner_node.cpp)
target_link_libraries(experiment_runner_node planner_experiments_core)

add_executable(trajectory_validator_node src/trajectory_validator_node.cpp)
target_link_libraries(trajectory_validator_node planner_experiments_core)

catkin_install_python(PROGRAMS
  scripts/run_batch.py
  scripts/generate_tasks.py
  scripts/generate_parametric_maps.py
  scripts/analyze_results.py
  scripts/check_results.py
  scripts/export_paper_tables.py
  scripts/replay_experiment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES scripts/_common.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_experiment_utils test/test_experiment_utils.cpp)
  if(TARGET test_experiment_utils)
    target_link_libraries(test_experiment_utils planner_experiments_core)
  endif()
  catkin_add_nosetests(test/test_python_utils.py)
endif()

install(TARGETS planner_experiments_core experiment_runner_node trajectory_validator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config datasets launch maps meshes rviz tasks
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

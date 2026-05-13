cmake_minimum_required(VERSION 3.5)
project(ufil_object_tracking)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED)

# Define the header-only library as an INTERFACE target
add_library(ufil_object_tracking INTERFACE)
add_library(ufil_object_tracking::ufil_object_tracking ALIAS ufil_object_tracking)

target_include_directories(ufil_object_tracking PUBLIC INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ufil_object_tracking>
)

# Link dependencies
target_link_libraries(ufil_object_tracking INTERFACE
  Eigen3::Eigen
  TBB::tbb
  Boost::boost
)

install(TARGETS ufil_object_tracking
  EXPORT "export_ufil_object_tracking"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ufil_object_tracking/
  DESTINATION include/ufil_object_tracking
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(ufil_object_tracking_hypothesize_functions_test test/hypothesize_functions_test.cpp)
  target_link_libraries(ufil_object_tracking_hypothesize_functions_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_hypothesizer_test test/hypothesizer_test.cpp)
  target_link_libraries(ufil_object_tracking_hypothesizer_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_association_functions_test test/association_functions_test.cpp)
  target_link_libraries(ufil_object_tracking_association_functions_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_time_test test/time_test.cpp)
  target_link_libraries(ufil_object_tracking_time_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_measurement_test test/measurement_test.cpp)
  target_link_libraries(ufil_object_tracking_measurement_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_scalar_test test/scalar_test.cpp)
  target_link_libraries(ufil_object_tracking_scalar_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_state_test test/state_test.cpp)
  target_link_libraries(ufil_object_tracking_state_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_id_test test/id_test.cpp)
  target_link_libraries(ufil_object_tracking_id_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_track_test test/track_test.cpp)
  target_link_libraries(ufil_object_tracking_track_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_length_pruner_test test/length_pruner_test.cpp)
  target_link_libraries(ufil_object_tracking_length_pruner_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_history_entry_test test/history_entry_test.cpp)
  target_link_libraries(ufil_object_tracking_history_entry_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_json_history_deleter_test test/json_history_deleter_test.cpp)
  target_link_libraries(ufil_object_tracking_json_history_deleter_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_json_serialization_test test/json_serialization_test.cpp)
  target_link_libraries(ufil_object_tracking_json_serialization_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_dimension_test test/dimension_test.cpp)
  target_link_libraries(ufil_object_tracking_dimension_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_control_test test/control_test.cpp)
  target_link_libraries(ufil_object_tracking_control_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_classification_test test/classification_test.cpp)
  target_link_libraries(ufil_object_tracking_classification_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_detection_test test/detection_test.cpp)
  target_link_libraries(ufil_object_tracking_detection_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_predictor_test test/predictor_test.cpp)
  target_link_libraries(ufil_object_tracking_predictor_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_random_walk_transition_model_test test/random_walk_transition_model_test.cpp)
  target_link_libraries(ufil_object_tracking_random_walk_transition_model_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_constant_velocity_transition_model_test test/constant_velocity_transition_model_test.cpp)
  target_link_libraries(ufil_object_tracking_constant_velocity_transition_model_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_constant_acceleration_transition_model_test test/constant_acceleration_transition_model_test.cpp)
  target_link_libraries(ufil_object_tracking_constant_acceleration_transition_model_test ufil_object_tracking)

  ament_add_gtest(ufil_object_tracking_dimension_gridmap_updater_test test/dimension_gridmap_updater_test.cpp)
  target_link_libraries(ufil_object_tracking_dimension_gridmap_updater_test ufil_object_tracking)

  ament_lint_auto_find_test_dependencies()
endif()

# Export the target
ament_export_include_directories("include/ufil_object_tracking")
ament_export_dependencies(Eigen3 TBB Boost)
ament_export_targets("export_ufil_object_tracking")

# Declare the ament package
ament_package()

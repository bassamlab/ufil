cmake_minimum_required(VERSION 3.8)
project(ufil_object_tracking VERSION 0.2.0)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search filters segmentation features)
find_package(TBB REQUIRED)

# Header-only library without ROS 2 dependencies.
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} INTERFACE
  Boost::boost
  Eigen3::Eigen
  pcl_common
  pcl_features
  pcl_filters
  pcl_search
  pcl_segmentation
  TBB::tbb
)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name
    hypothesize_functions_test
    hypothesizer_test
    association_functions_test
    time_test
    measurement_test
    scalar_test
    state_test
    id_test
    track_test
    length_pruner_test
    history_entry_test
    json_history_deleter_test
    json_serialization_test
    dimension_test
    control_test
    classification_test
    detection_test
    detection_pipeline_test
    predictor_test
    random_walk_transition_model_test
    constant_velocity_transition_model_test
    constant_acceleration_transition_model_test
    dimension_gridmap_updater_test
  )
    ament_add_gtest(${PROJECT_NAME}_${test_name} test/${test_name}.cpp)
    target_link_libraries(${PROJECT_NAME}_${test_name} ${PROJECT_NAME})
    target_compile_options(${PROJECT_NAME}_${test_name} PRIVATE
      $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -Wpedantic>)
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Boost Eigen3 PCL TBB)

ament_package()

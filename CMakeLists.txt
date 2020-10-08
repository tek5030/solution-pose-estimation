cmake_minimum_required( VERSION 3.5.1 )

project(lab_6_pose)

# Compiler flags
if (MSVC)
  set(compiler_options /W4)
  add_definitions(/D_USE_MATH_DEFINES)
else ()
  set(compiler_options -Wall -Wextra -Wpedantic)
endif (MSVC)

# Add libraries.
find_package(OpenCV 4 CONFIG REQUIRED)
find_package(Eigen3 3.3.4 CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)

add_executable(${PROJECT_NAME}
  lab_6.h
  lab_6.cpp
  ar_example.h
  ar_example.cpp
  feature_utils.h
  feature_utils.cpp
  homography_pose_estimator.h
  homography_pose_estimator.cpp
  main.cpp
  moba_pose_estimator.h
  moba_pose_estimator.cpp
  camera_projection_measurement.h
  camera_projection_measurement.cpp
  plane_world_model.h
  plane_world_model.cpp
  pnp_pose_estimator.h
  pnp_pose_estimator.cpp
  pose_estimator.h
  scene_3d.h
  scene_3d.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Sophus::Sophus
  )

target_compile_options(${PROJECT_NAME} PRIVATE ${compiler_options})
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 14
  )

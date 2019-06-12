cmake_minimum_required(VERSION 2.8.3)
project(aizuspider_description)

# find_package(catkin REQUIRED COMPONENTS
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs roseus)
#   hrpsys_ros_bridge
# )

# compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/aero_upper.wrl)
add_service_files(
  FILES Grasp.srv SolveIK.srv
)
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  DEPENDS roseus
  CATKIN_DEPENDS roseus geometry_msgs std_msgs message_runtime
  )


cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)
project(ik_solvers2_python)

#add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
#  fullbody_inverse_kinematics_solver
  prioritized_inverse_kinematics_solver2
  )
find_package(choreonoid REQUIRED)
#find_package(OsqpEigen REQUIRED)

catkin_package(
#  LIBRARIES fullbody_inverse_kinematics_solver
  CATKIN_DEPENDS prioritized_inverse_kinematics_solver2 ik_constraint2
  DEPENDS OsqpEigen
)

add_subdirectory(pybind11)

install(DIRECTORY sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

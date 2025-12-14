cmake_minimum_required(VERSION 3.8)
project(sauvc_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install folder launch dan urdf agar bisa dipanggil
install(DIRECTORY
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)
# Install Script Python
install(PROGRAMS
  scripts/thruster_manager.py
  scripts/auto_pilot.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

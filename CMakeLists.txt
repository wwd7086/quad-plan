cmake_minimum_required(VERSION 2.8.3)
project(sbpl_plan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_utils
  geometry_msgs
  nav_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_utils geometry_msgs nav_msgs
  DEPENDS system_lib sbpl
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${SBPL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/SbplPlanner.cpp src/grid2D.cpp)
target_link_libraries(${PROJECT_NAME}
 	${catkin_LIBRARIES}
 	${SBPL_LIBRARIES}
 )

add_executable(${PROJECT_NAME}_node src/sbpl_plan_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
	${PROJECT_NAME}
)

install(TARGETS sbpl_plan sbpl_plan_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
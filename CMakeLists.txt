cmake_minimum_required(VERSION 3.10.0)
project(subscription_notifier)


include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES subscription_notifier
  CATKIN_DEPENDS roscpp
  DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/subscription_notifier.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
cnr_target_compile_options(${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/subscription_notifier_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} )
cnr_target_compile_options(${PROJECT_NAME}_node)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


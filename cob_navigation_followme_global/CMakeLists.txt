cmake_minimum_required(VERSION 2.8.3)
project(cob_navigation_followme_global)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_core
            costmap_2d
            base_local_planner
            pluginlib
        )

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cob_navigation_followme_global
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
)

add_library(cob_navigation_followme_global src/cob_followme_global_planner.cpp)
target_link_libraries(cob_navigation_followme_global
    ${catkin_LIBRARIES}
    )

install(TARGETS cob_navigation_followme_global
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



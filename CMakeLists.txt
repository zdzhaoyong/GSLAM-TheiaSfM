cmake_minimum_required(VERSION 2.8)

include(cmake/PICMake.cmake)

if(THEIA_INSTALLED)
pi_add_target(theiaSfM SHARED TheiaSfM.cpp REQUIRED Theia GSLAM)
pi_install(TARGETS theiaSfM)
else()
pi_collect_packages(VERBOSE REQUIRED GSLAM Eigen3)
set(DISABLE_APPLICATIONS true)
set(BUILD_TESTING OFF)
add_subdirectory(TheiaSfM)
pi_add_target(theiaSfM SHARED TheiaSfM.cpp REQUIRED GSLAM Eigen3 DEPENDENCY theia)
endif()

pi_report_targets()

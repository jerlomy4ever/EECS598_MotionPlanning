cmake_minimum_required (VERSION 2.6.0)
project(myplugin)
find_package(OpenRAVE 0.9 REQUIRED)


if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

aux_source_directory(. DIR_SRCS)
add_subdirectory(alglib-3.16.0.cpp.gpl/cpp/src)
add_subdirectory(Head)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} )
add_library(myplugin SHARED myplugin.cpp)
set_target_properties(myplugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(myplugin ${OpenRAVE_LIBRARIES} )
target_link_libraries(myplugin StructSelfDefine)
target_link_libraries(myplugin alglib)
install(TARGETS myplugin DESTINATION .)


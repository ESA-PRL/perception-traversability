find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


rock_library(traversability
    SOURCES traversability.cpp
    HEADERS traversability.hpp
    DEPS_PKGCONFIG opencv)

rock_executable(traversability_bin Main.cpp
    DEPS traversability)

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

target_link_libraries(cartographer
  ${PCL_LIBRARIES}
  opencv_core)

rock_library(traversability
    SOURCES traversability.cpp
    HEADERS traversability.hpp
    DEPS_PKGCONFIG opencv)

target_link_libraries(traversability opencv_core)

rock_library(traversability
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(traversability_bin Main.cpp
    DEPS traversability)


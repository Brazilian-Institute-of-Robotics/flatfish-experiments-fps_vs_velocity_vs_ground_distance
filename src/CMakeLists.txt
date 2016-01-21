rock_library(fps_per_velocity
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(fps_per_velocity_bin Main.cpp
    DEPS fps_per_velocity)


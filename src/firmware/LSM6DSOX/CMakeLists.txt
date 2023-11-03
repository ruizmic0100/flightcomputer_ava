add_executable(arduino_lsm
        LSM6DSOX.cpp
        )

target_include_directories(arduino_lsm PUBLIC "arduino")

# pull in common dependencies
target_link_libraries(arduino_lsm "arduino")
add_executable(open_fc open_fc.cpp)

# pull in common dependencies
target_link_libraries(open_fc pico_stdlib pico_multicore)

# enable usb output, disable uart output
pico_enable_stdio_usb(open_fc 1)
pico_enable_stdio_uart(open_fc 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(open_fc)

# add url via pico_set_program_url
example_auto_set_url(open_fc)

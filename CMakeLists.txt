add_executable(bittenpico
        src/bittenpico.c
        )

# pull in common dependencies
target_link_libraries(bittenpico pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(bittenpico)

# add url via pico_set_program_url
example_auto_set_url(bittenpico)
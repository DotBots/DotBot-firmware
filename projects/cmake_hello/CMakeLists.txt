# cmake_hello — minimal LED blinker; the canonical "did the CMake
# build pipeline come together?" smoke test.

# Startup, vector table, and system_init all come from dotbot-libs
# (nRF/System/cpu.c + nRF5340_xxAA_Application_cpu.c + system_init.c),
# pulled in via the `dotbot_libs` INTERFACE sources. No .S file needed
# — aabadie wrote the whole startup in C. The linker just needs the
# matching script.
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/cmake/targets/nrf5340-app/nrf5340_app.ld)

add_executable(cmake_hello main.c)

target_link_libraries(cmake_hello PRIVATE dotbot_libs)
# Nordic MDK + ARM CMSIS-Core headers come transitively via dotbot_libs
# (which inherits from dotbot_nordic) — no need to list them here.

target_compile_definitions(cmake_hello PRIVATE
    NRF5340_XXAA
    NRF_APPLICATION
    __NRF_FAMILY
    BOARD_DOTBOT_V3
)

target_link_options(cmake_hello PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/cmake_hello.map
)

set_target_properties(cmake_hello PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

add_custom_command(TARGET cmake_hello POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:cmake_hello> ${CMAKE_CURRENT_BINARY_DIR}/cmake_hello.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:cmake_hello> ${CMAKE_CURRENT_BINARY_DIR}/cmake_hello.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:cmake_hello>
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/cmake_hello.hex
               ${CMAKE_CURRENT_BINARY_DIR}/cmake_hello.bin
    COMMENT "cmake_hello: hex + bin"
)

cmake_minimum_required(VERSION 3.16)
project(opendroneid_linux C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build type default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-address-of-packed-member)

# ── Submodule paths ───────────────────────────────────────────────────────────
set(CORE_C_DIR    ${CMAKE_SOURCE_DIR}/lib/opendroneid-core-c)
set(MAVLINK_DIR   ${CMAKE_SOURCE_DIR}/lib/mavlink)
set(INIH_DIR      ${CMAKE_SOURCE_DIR}/lib/inih)

# ── opendroneid-core-c ────────────────────────────────────────────────────────
# Build just the core library (not the extra wifi/nan parts)
add_library(opendroneid_core STATIC
    ${CORE_C_DIR}/libopendroneid/opendroneid.c
)
target_include_directories(opendroneid_core PUBLIC
    ${CORE_C_DIR}/libopendroneid
    ${CORE_C_DIR}
)
target_compile_options(opendroneid_core PRIVATE -w)  # suppress submodule warnings

# ── inih ─────────────────────────────────────────────────────────────────────
add_library(inih STATIC
    ${INIH_DIR}/ini.c
)
target_include_directories(inih PUBLIC ${INIH_DIR})
target_compile_options(inih PRIVATE -w)

# ── System libraries ──────────────────────────────────────────────────────────
find_library(BLUETOOTH_LIB bluetooth REQUIRED)
find_library(NL3_LIB       nl-3     REQUIRED)
find_library(NL_GENL3_LIB  nl-genl-3 REQUIRED)

# ── MAVLink include path ──────────────────────────────────────────────────────
# MAVLink is header-only; we need ardupilotmega dialect
set(MAVLINK_INCLUDE ${MAVLINK_DIR}/message_definitions/v1.0)
# The generated C headers live under the mavlink submodule root
# Standard path after running mavgenerate: lib/mavlink/ardupilotmega/
# We also support the pre-generated headers shipped in pymavlink or mavlink2
set(MAVLINK_C_INCLUDE ${MAVLINK_DIR})

# ── Main daemon ───────────────────────────────────────────────────────────────
add_executable(odid-daemon
    src/main.c
    src/config.c
    src/mavlink_reader.c
    src/odid_state.c
    src/bt_advertiser.c
    src/wifi_beacon.c
    src/utils.c
)

target_include_directories(odid-daemon PRIVATE
    src/
    ${MAVLINK_C_INCLUDE}
    ${MAVLINK_C_INCLUDE}/ardupilotmega   # dialect-specific headers
)

target_include_directories(odid-daemon PRIVATE /usr/include/libnl3)

target_link_libraries(odid-daemon PRIVATE
    opendroneid_core
    inih
    ${BLUETOOTH_LIB}
    ${NL3_LIB}
    ${NL_GENL3_LIB}
    pthread
    m
)

# ── Tests ─────────────────────────────────────────────────────────────────────
enable_testing()

add_executable(test_encoding
    tests/test_encoding.c
)
target_include_directories(test_encoding PRIVATE
    src/
    ${MAVLINK_C_INCLUDE}
    ${MAVLINK_C_INCLUDE}/ardupilotmega
)
target_link_libraries(test_encoding PRIVATE
    opendroneid_core
    inih
    pthread m
)
add_test(NAME encoding COMMAND test_encoding)

add_executable(test_config
    tests/test_config.c
    src/config.c
    src/utils.c
)
target_include_directories(test_config PRIVATE src/)
target_link_libraries(test_config PRIVATE inih)
add_test(NAME config COMMAND test_config)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS odid-daemon DESTINATION bin)
install(FILES odid.conf.example DESTINATION /etc/odid RENAME odid.conf.example)
install(FILES systemd/odid.service DESTINATION /etc/systemd/system)

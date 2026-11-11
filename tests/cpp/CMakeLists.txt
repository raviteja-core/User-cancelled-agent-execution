include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(unit_tests
    test_crc16.cpp
    test_protocol.cpp
    test_transport.cpp
    test_fsm.cpp
    test_motion.cpp
    test_sim.cpp
    test_safety.cpp
    ../../controller_app/transport_mem.cpp
)

target_include_directories(unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../controller_app
)

target_link_libraries(unit_tests PRIVATE
    papercutter_core
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)

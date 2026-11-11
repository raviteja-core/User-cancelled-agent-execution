add_library(papercutter_core STATIC
    src/protocol/crc16.cpp
    src/protocol/encoder.cpp
    src/protocol/parser.cpp
)

target_include_directories(papercutter_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(papercutter_core PUBLIC cxx_std_17)

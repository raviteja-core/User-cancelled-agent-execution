add_executable(controller_app
    main.cpp
    transport_mem.cpp
    transport_serial_win.cpp
    transport_tcp.cpp
)

target_include_directories(controller_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
)

target_link_libraries(controller_app PRIVATE
    papercutter_core
    ws2_32
)

target_compile_features(controller_app PRIVATE cxx_std_17)

configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(PandaProsthesis module.cpp module.h)
target_link_libraries(PandaProsthesis PUBLIC mc_panda::panda)
target_include_directories(PandaProsthesis PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

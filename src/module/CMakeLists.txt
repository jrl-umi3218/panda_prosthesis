# Attempt to find extra urdf for extra robot modules
find_package(catkin QUIET)
if(${catkin_FOUND})
  message(STATUS "catkin found, looking for extra description packages")
  find_package(catkin QUIET COMPONENTS brace_bottom_setup brace_top_setup)
  if(${brace_bottom_setup_FOUND})
    # brace_bottom_setup_SOURCE_PREFIX, brace_bottom_setup_DEVEL_PREFIX, brace_bottom_setup_INSTALL_PREFIX
    message(STATUS "Found brace_bottom_setup sources in ${brace_bottom_setup_SOURCE_PREFIX}")
    set(brace_bottom_setup_DIR ${brace_bottom_setup_SOURCE_PREFIX})

    configure_file(brace_bottom_setup.in.yml ${CMAKE_CURRENT_BINARY_DIR}/brace_bottom_setup.yml)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/brace_bottom_setup.yml DESTINATION ${MC_ROBOTS_ALIASES_DIRECTORY})
  endif()

  if(${brace_top_setup_FOUND})
    # brace_top_setup_SOURCE_PREFIX, brace_top_setup_DEVEL_PREFIX, brace_top_setup_INSTALL_PREFIX
    message(STATUS "Found brace_top_setup sources in ${brace_top_setup_SOURCE_PREFIX}")
    set(brace_top_setup_DIR ${brace_top_setup_SOURCE_PREFIX})

    configure_file(brace_top_setup.in.yml ${CMAKE_CURRENT_BINARY_DIR}/brace_top_setup.yml)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/brace_top_setup.yml DESTINATION ${MC_ROBOTS_ALIASES_DIRECTORY})
  endif()
endif()


configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(PandaProsthesisModule module.cpp module.h)
target_link_libraries(PandaProsthesisModule PUBLIC mc_panda::panda)
target_include_directories(PandaProsthesisModule PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

# Attempt to find extra urdf for extra robot modules
find_package(catkin QUIET)
if(${catkin_FOUND})
  message(STATUS "catkin found, looking for extra description packages")
  find_package(catkin REQUIRED COMPONENTS bottom10)
  if(${bottom10_FOUND})
    # bottom10_SOURCE_PREFIX, bottom10_DEVEL_PREFIX, bottom10_INSTALL_PREFIX
    message(STATUS "Found bottom10 sources in ${bottom10_SOURCE_PREFIX}")
    set(bottom10_DIR ${bottom10_SOURCE_PREFIX})

    configure_file(panda_prosthesis_aliases.in.yml ${CMAKE_CURRENT_BINARY_DIR}/panda_prosthesis_aliases.yml)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/panda_prosthesis_aliases.yml DESTINATION ${MC_ROBOTS_ALIASES_DIRECTORY})
  endif()
endif()


configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(PandaProsthesisModule module.cpp module.h)
target_link_libraries(PandaProsthesisModule PUBLIC mc_panda::panda)
target_include_directories(PandaProsthesisModule PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")
add_robot(PandaProsthesisModule module.cpp module.h)
target_link_libraries(PandaProsthesisModule PUBLIC mc_panda::panda)
target_include_directories(PandaProsthesisModule PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

# Attempt to find extra urdf for extra robot modules
find_package(catkin QUIET)
if(catkin_FOUND)
  message(STATUS "catkin found, looking for extra description packages")
  find_package(catkin QUIET COMPONENTS brace_bottom_setup brace_top_setup brace_top_setup_brace brace_urdf)
else() # ROS2
  message(STATUS "Building using ROS2")
  find_package(brace_bottom_setup QUIET)
  find_package(brace_top_setup QUIET)
  find_package(brace_top_setup_brace QUIET)
  find_package(brace_urdf QUIET)
endif()

if(brace_bottom_setup_FOUND)
  # brace_bottom_setup_SOURCE_PREFIX, brace_bottom_setup_DEVEL_PREFIX, brace_bottom_setup_INSTALL_PREFIX
  message(STATUS "Found brace_bottom_setup sources in ${brace_bottom_setup_SOURCE_PREFIX}")
  set(brace_bottom_setup_DIR ${brace_bottom_setup_SOURCE_PREFIX})

  # Generate the robot module YAML description using this value
  configure_file(yaml/brace_bottom_setup.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/yaml/brace_bottom_setup.yaml")

  # Install the YAML file
  set(MODULE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml/brace_bottom_setup.yaml" DESTINATION "${MODULE_INSTALL_LOCATION}")

  # Configure the alias accordingly
  configure_file(alias/brace_bottom_setup.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/alias/brace_bottom_setup.yaml")

  # Install the alias
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias/brace_bottom_setup.yaml" DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")
endif()

if(brace_urdf_FOUND)
  # brace_urdf_SOURCE_PREFIX, brace_urdf_DEVEL_PREFIX, brace_urdf_INSTALL_PREFIX
  message(STATUS "Found brace_urdf sources in ${brace_urdf_SOURCE_PREFIX}")
  set(brace_urdf_DIR ${brace_urdf_SOURCE_PREFIX})

  # Generate the robot module YAML description using this value
  configure_file(yaml/brace_urdf.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/yaml/brace_urdf.yaml")

  # Install the YAML file
  set(MODULE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml/brace_urdf.yaml" DESTINATION "${MODULE_INSTALL_LOCATION}")

  # Configure the alias accordingly
  configure_file(alias/brace_urdf.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/alias/brace_urdf.yaml")

  # Install the alias
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias/brace_urdf.yaml" DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")
endif()

if(brace_top_setup_FOUND)
  # brace_top_setup_SOURCE_PREFIX, brace_top_setup_DEVEL_PREFIX, brace_top_setup_INSTALL_PREFIX
  message(STATUS "Found brace_top_setup sources in ${brace_top_setup_SOURCE_PREFIX}")
  set(brace_top_setup_DIR ${brace_top_setup_SOURCE_PREFIX})

  configure_file(alias/brace_top_setup.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/alias/brace_top_setup.yaml)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/alias/brace_top_setup.yaml DESTINATION ${MC_ROBOTS_ALIASES_DIRECTORY})
else()
  message(WARNING "brace_top_setup not found")
endif()

if(brace_top_setup_brace_FOUND)
  # brace_top_setup_brace_SOURCE_PREFIX, brace_top_setup_brace_DEVEL_PREFIX, brace_top_setup_brace_INSTALL_PREFIX
  message(STATUS "Found brace_top_setup_brace sources in ${brace_top_setup_brace_SOURCE_PREFIX}")
  set(brace_top_setup_brace_DIR ${brace_top_setup_brace_SOURCE_PREFIX})

  configure_file(alias/brace_top_setup_brace.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/alias/brace_top_setup_brace.yaml)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/alias/brace_top_setup_brace.yaml DESTINATION ${MC_ROBOTS_ALIASES_DIRECTORY})
else()
  message(WARNING "brace_top_setup_brace not found")
endif()

if(brace_top_setup_FOUND AND brace_top_setup_brace_FOUND)
  message(STATUS "Found both brace_top_setup and brace_top_setup_brace, building PandaBrace robot module")
  configure_file(config_panda_brace.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config_panda_brace.h")
  add_robot(PandaBraceModule panda_brace_module.cpp panda_brace_module.h)
  target_link_libraries(PandaBraceModule PUBLIC mc_panda::panda)
  target_include_directories(PandaBraceModule PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

  install(FILES etc/ForceSensorCalibration/panda_brace_femur.yaml DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/ForceSensorCalibration/)
endif()

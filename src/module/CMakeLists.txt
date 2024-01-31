configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")
add_robot(PandaProsthesisModule module.cpp module.h)
target_link_libraries(PandaProsthesisModule PUBLIC mc_panda::panda)
target_include_directories(PandaProsthesisModule PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

find_description_package(brace_bottom_setup)
if(brace_bottom_setup_FOUND)
  set(brace_bottom_setup_DIR ${BRACE_BOTTOM_SETUP_PATH})
  message(STATUS "Found brace_bottom_setup sources in ${brace_bottom_setup_DIR}")

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

find_description_package(brace_urdf)
if(brace_urdf_FOUND)
  set(brace_urdf_DIR ${BRACE_URDF_DESCRIPTION_PATH})
  message(STATUS "Found brace_urdf sources in ${brace_urdf_DIR}")

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

find_description_package(brace_top_setup)
if(brace_top_setup_FOUND)
  set(brace_top_setup_DIR ${BRACE_TOP_SETUP_DESCRIPTION_PATH})
  message(STATUS "Found brace_top_setup sources in ${brace_top_setup_DIR}")

  configure_file(alias/brace_top_setup.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/alias/brace_top_setup.yaml)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/alias/brace_top_setup.yaml DESTINATION ${MC_ROBOTS_ALIASES_DIRECTORY})
else()
  message(WARNING "brace_top_setup not found")
endif()

find_description_package(brace_top_setup_brace)
if(brace_top_setup_brace_FOUND)
  set(brace_top_setup_brace_DIR ${BRACE_TOP_SETUP_BRACE_DESCRIPTION_PATH})
  message(STATUS "Found brace_top_setup_brace sources in ${brace_top_setup_brace_DIR}")

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

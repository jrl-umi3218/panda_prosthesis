cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
project(panda_prosthesis LANGUAGES CXX VERSION 1.0.0)

find_package(mc_panda REQUIRED)


set(DATA_DIR "${CMAKE_INSTALL_PREFIX}/share/panda_prosthesis")
set(calib_DIR "${DATA_DIR}/calib")
set(meshes_DIR "${DATA_DIR}/meshes")
set(convex_DIR "${DATA_DIR}/convex")
set(inertia_DIR "${DATA_DIR}/inertia")
set(transforms_DIR "${DATA_DIR}/transforms")
set(rsdf_DIR "${DATA_DIR}/rsdf")

install(DIRECTORY meshes DESTINATION "${DATA_DIR}" FILES_MATCHING PATTERN "*.stl")
install(DIRECTORY convex DESTINATION "${DATA_DIR}" FILES_MATCHING PATTERN "*-ch.txt")
install(DIRECTORY inertia DESTINATION "${DATA_DIR}" FILES_MATCHING PATTERN "*.yml")
install(DIRECTORY transforms DESTINATION "${DATA_DIR}" FILES_MATCHING PATTERN "*.yml")
install(DIRECTORY rsdf DESTINATION "${DATA_DIR}" FILES_MATCHING PATTERN "*.rsdf")

add_subdirectory(src)


set(BONETAG_CONTROLLER_ETC "${PROJECT_SOURCE_DIR}/src/controller/etc")
configure_file(scripts/run_bonetag_controller_real.in.sh "${CMAKE_CURRENT_BINARY_DIR}/scripts/run_bonetag_controller_real" @ONLY)
configure_file(scripts/run_bonetag_controller_simu.in.sh "${CMAKE_CURRENT_BINARY_DIR}/scripts/run_bonetag_controller_simu" @ONLY)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/scripts/run_bonetag_controller_real
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/scripts/run_bonetag_controller_simu
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

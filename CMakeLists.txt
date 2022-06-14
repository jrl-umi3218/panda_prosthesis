cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
project(panda_prosthesis LANGUAGES CXX VERSION 1.0.0)

find_package(mc_panda REQUIRED)


set(DATA_DIR "${CMAKE_INSTALL_PREFIX}/share/panda_prosthesis")
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

set(TRAJECTORY_DIR "${PROJECT_SOURCE_DIR}/data/trajectories")
set(RESULTS_DIR "${PROJECT_SOURCE_DIR}/data/results")
add_subdirectory(src)

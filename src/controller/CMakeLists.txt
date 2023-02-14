set(common_SRC
  src/trajectory.cpp
)
set(common_HDR
  include/trajectory.h
  include/utils.h
)
add_library(PandaProsthesisCommon SHARED ${common_SRC} ${common_HDR})
target_include_directories(PandaProsthesisCommon PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(PandaProsthesisCommon PUBLIC SpaceVecAlg::SpaceVecAlg)
install(TARGETS PandaProsthesisCommon
  LIBRARY DESTINATION lib)

# PandaProsthesis controller (BoneTag)
add_controller(PandaProsthesisController PandaProsthesisController.cpp PandaProsthesisController.h)
target_link_libraries(PandaProsthesisController PUBLIC PandaProsthesisCommon)
target_link_libraries(PandaProsthesisController PUBLIC mc_rtc::mc_control_fsm)
target_include_directories(PandaProsthesisController PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../module/include>)

set(TRAJECTORY_DIR "${PROJECT_SOURCE_DIR}/data/trajectories/bonetag")
set(RESULTS_DIR "${PROJECT_SOURCE_DIR}/data/results/bonetag")
configure_file(etc/PandaProsthesis.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/PandaProsthesis.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/PandaProsthesis.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")


# PandaBrace controller (Lea's orthesis)
if(TARGET PandaBraceModule)
  message(STATUS "Will build PandaBraceController as the PandaBraceModule is available")
  add_controller(PandaBraceController PandaBraceController.cpp PandaBraceController.h)
  target_link_libraries(PandaBraceController PUBLIC PandaProsthesisCommon)
  target_link_libraries(PandaBraceController PUBLIC mc_rtc::mc_control_fsm)
  target_include_directories(PandaBraceController PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../module/include>)

  set(TRAJECTORY_DIR "${PROJECT_SOURCE_DIR}/data/trajectories/brace")
  set(RESULTS_DIR "${PROJECT_SOURCE_DIR}/data/results/brace")
  configure_file(etc/PandaBrace.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/PandaBrace.yaml")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/PandaBrace.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
endif()



add_subdirectory(states)

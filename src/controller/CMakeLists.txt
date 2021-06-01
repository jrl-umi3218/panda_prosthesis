add_controller(PandaProsthesisController controller.cpp controller.h)
target_link_libraries(PandaProsthesisController PUBLIC mc_rtc::mc_control_fsm)

configure_file(etc/PandaProsthesis.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/PandaProsthesis.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/PandaProsthesis.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

add_subdirectory(states)

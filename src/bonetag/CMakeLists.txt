add_library(BoneTagSerial SHARED BoneTagSerial.cpp)
target_link_libraries(BoneTagSerial PUBLIC mc_rtc::mc_rtc_utils)

install(TARGETS BoneTagSerial
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_executable(TestBoneTagUSB test_bonetag_usb.cpp)
target_link_libraries(TestBoneTagUSB PUBLIC BoneTagSerial)



set(AUTOLOAD_BoneTagSerialPlugin_PLUGIN false CACHE BOOL "When true the plugin will always be loaded")

set(plugin_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/BoneTagSerialPlugin.cpp
)

set(plugin_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/BoneTagSerialPlugin.h
)
add_plugin(BoneTagSerialPlugin "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(BoneTagSerialPlugin PUBLIC BoneTagSerial)
install(FILES etc/BoneTagSerialPlugin.yaml DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)

install(
  FILES scripts/reset_bonetag_serial
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE
)


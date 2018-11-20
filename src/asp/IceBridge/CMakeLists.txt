
# IceBridge

add_executable(correct_icebridge_l3_dem correct_icebridge_l3_dem.cc)
target_link_libraries(correct_icebridge_l3_dem AspCore)
install(TARGETS correct_icebridge_l3_dem DESTINATION bin)

add_executable(nav2cam nav2cam.cc)
target_link_libraries(nav2cam AspCore)
install(TARGETS nav2cam DESTINATION bin)

add_executable(ortho2pinhole ortho2pinhole.cc)
target_link_libraries(ortho2pinhole AspSessions)
install(TARGETS ortho2pinhole DESTINATION bin)

add_executable(orbitviz_pinhole orbitviz_pinhole.cc)
target_link_libraries(orbitviz_pinhole AspSessions)
install(TARGETS orbitviz_pinhole DESTINATION bin)

add_executable(qi2txt qi2txt.cc)
target_link_libraries(qi2txt AspCore)
install(TARGETS qi2txt DESTINATION bin)

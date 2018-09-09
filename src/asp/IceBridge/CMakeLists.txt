
# IceBridge

add_executable(correct_icebridge_l3_dem correct_icebridge_l3_dem.cc) 
target_link_libraries(correct_icebridge_l3_dem ${FULL_LIBRARY_LIST}) 
install(TARGETS correct_icebridge_l3_dem DESTINATION bin)

add_executable(nav2cam nav2cam.cc) 
target_link_libraries(nav2cam ${FULL_LIBRARY_LIST}) 
install(TARGETS nav2cam DESTINATION bin)

add_executable(orbitviz_pinhole orbitviz_pinhole.cc) 
target_link_libraries(orbitviz_pinhole ${FULL_LIBRARY_LIST}) 
install(TARGETS orbitviz_pinhole DESTINATION bin)

add_executable(nav2cam nav2cam.cc) 
target_link_libraries(nav2cam ${FULL_LIBRARY_LIST}) 
install(TARGETS nav2cam DESTINATION bin)

add_executable(qi2txt qi2txt.cc) 
target_link_libraries(qi2txt ${FULL_LIBRARY_LIST}) 
install(TARGETS qi2txt DESTINATION bin)

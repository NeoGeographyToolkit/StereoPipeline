
# IceBridge

add_executable(correct_icebridge_l3_dem correct_icebridge_l3_dem.cc)
target_link_libraries(correct_icebridge_l3_dem AspCore AspSessions)
install(TARGETS correct_icebridge_l3_dem DESTINATION bin)

add_executable(nav2cam nav2cam.cc)
target_link_libraries(nav2cam AspCore AspSessions)
install(TARGETS nav2cam DESTINATION bin)

add_executable(ortho2pinhole ortho2pinhole.cc)
target_link_libraries(ortho2pinhole AspSessions)
install(TARGETS ortho2pinhole DESTINATION bin)

add_executable(orbitviz_pinhole orbitviz_pinhole.cc)
target_link_libraries(orbitviz_pinhole AspSessions)
install(TARGETS orbitviz_pinhole DESTINATION bin)

# Install all of the python files.
set(PYTHON_TOOLS icebridge_kmz_to_csv.py extract_icebridge_ATM_points.py lvis2kml.py)

foreach(p ${PYTHON_TOOLS})
  INSTALL(FILES ${p}
          PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE GROUP_READ WORLD_EXECUTE
          DESTINATION bin)
endforeach()

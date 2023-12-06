
# Hidden

add_executable(csv_filter csv_filter.cc)
target_link_libraries(csv_filter AspCore AspSessions)
install(TARGETS csv_filter DESTINATION bin)

add_executable(s2p_fltr s2p_fltr.cc)
target_link_libraries(s2p_fltr AspCore AspSessions)
install(TARGETS s2p_fltr DESTINATION bin)

add_executable(s2p_pprc s2p_pprc.cc)
target_link_libraries(s2p_pprc AspCore AspSessions)
install(TARGETS s2p_pprc DESTINATION bin)

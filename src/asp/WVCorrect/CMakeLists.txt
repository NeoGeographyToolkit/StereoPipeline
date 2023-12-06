add_executable(disp_avg disp_avg.cc)
target_link_libraries(disp_avg AspCore AspSessions)
install(TARGETS disp_avg DESTINATION bin)


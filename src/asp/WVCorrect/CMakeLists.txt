
# WVCorrect

# Not much code to compile in this directory

add_executable(disp_avg disp_avg.cc) 
target_link_libraries(disp_avg ${FULL_LIBRARY_LIST}) 
install(TARGETS disp_avg DESTINATION bin)


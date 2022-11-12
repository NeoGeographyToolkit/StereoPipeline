# Use wrapper function at this level to avoid code duplication
add_library_wrapper(AspGUI "${ASP_GUI_SRC_FILES}" "${ASP_GUI_TEST_FILES}" "${ASP_GUI_LIB_DEPENDENCIES}")

# Need this for QT to work.
set_target_properties(AspGUI PROPERTIES AUTOMOC TRUE)

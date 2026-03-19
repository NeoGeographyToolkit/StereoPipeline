# Use wrapper function at this level to avoid code duplication
add_library_wrapper(AspSfmView "${ASP_SFMVIEW_SRC_FILES}" "${ASP_SFMVIEW_TEST_FILES}" "${ASP_SFMVIEW_LIB_DEPENDENCIES}")

# Need this for QT to work.
set_target_properties(AspSfmView PROPERTIES AUTOMOC TRUE)

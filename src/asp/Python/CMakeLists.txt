
# Python

# Install all of the python files.
set(PYTHON_FILES asp_alg_utils.py   asp_image_utils.py
                 asp_cmd_utils.py   asp_string_utils.py
                 asp_file_utils.py  asp_system_utils.py
                 asp_geo_utils.py)

foreach(p ${PYTHON_FILES})
  INSTALL(FILES ${p}  DESTINATION libexec)
endforeach()

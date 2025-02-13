
macro(patch_depends_variables)
  get_cmake_property(all_vars VARIABLES)

  foreach(var ${all_vars})
    if(var MATCHES ".*_DEPENDS$")
      set(var_value "${${var}}")

      # message(STATUS "#### Found depends ${var}")
      if(var_value MATCHES ".*boost.*")
        string(REPLACE "boost" "Boost" new_value "${var_value}")
        set(${var} "${new_value}")
        # message(STATUS "#### Updated ${var}: ${${var}}")
      endif()
    endif()
  endforeach()
endmacro()

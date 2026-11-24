# Copyright 2025 Arduino SA
# SPDX-License-Identifier: Apache-2.0

# Helper of misc/llext_edk/CMakeLists.txt, run as the "compiler" of the
# llext_edk_build_props target:
#
#   cmake -DOUT_DIR=<dir> -DOBJECT=<object> -P write_build_props.cmake --
#         DEFINES <defines...> INCLUDES <includes...> FLAGS <flags...>
#
# Writes c_defs.txt, c_incs.txt and c_flags.txt to OUT_DIR, with the arguments of
# each group separated by spaces, as the compiler command line has them (double
# quotes escaped again with a backslash), and creates an empty OBJECT.

set(defs)
set(incs)
set(flags)
set(group)
math(EXPR last "${CMAKE_ARGC} - 1")
foreach(i RANGE ${last})
  set(arg "${CMAKE_ARGV${i}}")
  if(NOT group)
    if(arg STREQUAL "DEFINES")
      set(group defs)
    endif()
  elseif(arg STREQUAL "DEFINES")
    set(group defs)
  elseif(arg STREQUAL "INCLUDES")
    set(group incs)
  elseif(arg STREQUAL "FLAGS")
    set(group flags)
  else()
    string(REPLACE "\"" "\\\"" arg "${arg}")
    string(APPEND ${group} " ${arg}")
  endif()
endforeach()

foreach(group defs incs flags)
  string(STRIP "${${group}}" value)
  file(WRITE ${OUT_DIR}/c_${group}.txt "${value}\n")
endforeach()
file(TOUCH ${OBJECT})

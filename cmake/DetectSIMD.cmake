# Detect the preprocessor directives which are set by the compiler.
execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dM -E -x c /dev/null
                OUTPUT_VARIABLE PREPROCESSOR_DIRECTIVES)

if (PREPROCESSOR_DIRECTIVES MATCHES "__SSE2__")
  add_definitions(-mssse3)
# For both armv7 and armv8, __ARM_NEON is used as preprocessor directive.
elseif (PREPROCESSOR_DIRECTIVES MATCHES "__ARM_ARCH 7")
  add_definitions(-mfpu=neon) # Needs to be set for armv7.
elseif (PREPROCESSOR_DIRECTIVES MATCHES "__ARM_ARCH 8")
  # Add potential compile flags for armv8 here.
endif()

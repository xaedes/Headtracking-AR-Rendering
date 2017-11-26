# Produce the final Version.h using template Version.h.in and substituting variables.
# We don't want to polute our source tree with it, thus putting it in binary tree.
configure_file("version.h.in" "${CMAKE_CURRENT_BINARY_DIR}/version.h" @ONLY)

# define publicly available header files
set(HEADER_FILES

)

# expose header files to other projects
foreach(header ${HEADER_FILES})
	configure_file("${header}" "${CMAKE_CURRENT_BINARY_DIR}/${header}" COPYONLY)
endforeach(header)

list(APPEND HEADER_FILES version.h)
set (HEADER_FILES ${HEADER_FILES} PARENT_SCOPE) 

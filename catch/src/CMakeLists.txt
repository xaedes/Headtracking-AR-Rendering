# Define two variables in order not to repeat ourselves.
set(LIBRARY_NAME "${PROJECT_NAME}")

prepend(LIB_HEADERS "${CMAKE_CURRENT_BINARY_DIR}/../include/${PROJECT_NAME}" ${HEADER_FILES})

set(LIB_SOURCES
	version.cpp
)


# Define the library
add_library(${LIBRARY_NAME} SHARED ${LIB_SOURCES})

# Set the build version. It will be used in the name of the lib, with corresponding
# symlinks created. SOVERSION could also be specified for api version. 
set_target_properties(${LIBRARY_NAME} PROPERTIES
    VERSION ${VERSION}  # ${VERSION} was defined in the main CMakeLists.
    FRAMEWORK FALSE
    PUBLIC_HEADER "${LIB_HEADERS}" # specify the public headers
)

# expose header files to other libraries
target_include_directories(${LIBRARY_NAME}
    PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/../include/
)

configure_file("test.sh" "${BIN_DIR}/test.sh" COPYONLY)

set (LIBRARY_NAME ${LIBRARY_NAME} PARENT_SCOPE) 

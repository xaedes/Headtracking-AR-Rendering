# Define two variables in order not to repeat ourselves.
set(LIBRARY_NAME "${PROJECT_NAME}")
set(APPLICATION_NAME "${PROJECT_NAME}_run")
set(APPLICATION2_NAME "${PROJECT_NAME}2_run")

prepend(LIB_HEADERS "${CMAKE_CURRENT_BINARY_DIR}/../include/${PROJECT_NAME}" ${HEADER_FILES})

set(LIB_SOURCES
	version.cpp
)

set(APP_HEADERS
)

set(APP_SOURCES
    main.cpp
)


set(APP2_HEADERS
)

set(APP2_SOURCES
    viewerMain.cpp
    Viewer.cpp
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

# Links the target library against the library (libprojBA)
target_link_libraries(${LIBRARY_NAME} common)

# This is a make target, so you can do a "make projBrunner"
add_executable(${APPLICATION_NAME} ${APP_SOURCES})
add_executable(${APPLICATION2_NAME} ${APP2_SOURCES})

# Links the target exe (projBrunner) against the library (libprojB) and other required libraries
target_link_libraries(${APPLICATION_NAME} ${LIBRARY_NAME}  )
target_link_libraries(${APPLICATION2_NAME} ${LIBRARY_NAME}  )
# target_link_libraries(${APPLICATION_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY} )

# add_library(OpenNI2 STATIC IMPORTED)
# set_target_properties(OpenNI2 PROPERTIES IMPORTED_LOCATION ${OPENNI2_LIBRARIES})
# target_link_libraries(${APPLICATION_NAME} OpenNI2 )
# target_link_libraries(${APPLICATION2_NAME} OpenNI2 )

target_link_libraries(${APPLICATION2_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

target_link_libraries(${APPLICATION_NAME} ${OPENNI2_LIBRARIES} )
target_link_libraries(${APPLICATION2_NAME} ${OPENNI2_LIBRARIES} )

include_directories ($<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>)
include_directories (${OPENNI2_INCLUDE_DIRS})

# expose header files to other libraries
target_include_directories(${LIBRARY_NAME}
    PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/../include/
)

set (LIBRARY_NAME ${LIBRARY_NAME} PARENT_SCOPE) 


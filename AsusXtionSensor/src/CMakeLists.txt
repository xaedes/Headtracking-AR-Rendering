# Define two variables in order not to repeat ourselves.
set(LIBRARY_NAME "${PROJECT_NAME}")
set(APPLICATION_NAME "${PROJECT_NAME}_run")

prepend(LIB_HEADERS "${CMAKE_CURRENT_BINARY_DIR}/../include/${PROJECT_NAME}" ${HEADER_FILES})

set(LIB_SOURCES
    version.cpp
	AsusXtionSensor.cpp
)

set(APP_SOURCES
    main.cpp
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
target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS})
target_link_libraries(${LIBRARY_NAME} ${OPENNI2_LIBRARIES} )

add_executable(${APPLICATION_NAME} ${APP_SOURCES})
target_link_libraries(${APPLICATION_NAME} ${LIBRARY_NAME})

include_directories ($<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>)
include_directories (${OPENNI2_INCLUDE_DIRS})

# expose header files to other libraries
target_include_directories(${LIBRARY_NAME}
    PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/../include/
)

set (LIBRARY_NAME ${LIBRARY_NAME} PARENT_SCOPE) 


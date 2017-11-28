# Define two variables in order not to repeat ourselves.
set(LIBRARY_NAME "${PROJECT_NAME}")
set(APPLICATION_NAME "${PROJECT_NAME}_run")

prepend(LIB_HEADERS "${CMAKE_CURRENT_BINARY_DIR}/../include/${PROJECT_NAME}" ${HEADER_FILES})

set(LIB_SOURCES
	version.cpp
)

set(APP_HEADERS
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
target_include_directories(${LIBRARY_NAME} PUBLIC $<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>)

# OpenFace
target_link_libraries(${LIBRARY_NAME} ${OpenFace_LIBRARIES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${OpenFace_INCLUDE_DIRS})

# dependencies of OpenFace.. so unclean
target_link_libraries(${LIBRARY_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${TBB_LIBRARIES} ${BLAS_LIBRARIES})
target_link_libraries(${LIBRARY_NAME} dlib)
target_include_directories(${LIBRARY_NAME} PUBLIC ${BLAS_INCLUDE_DIRS})
target_include_directories(${LIBRARY_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(${LIBRARY_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(${LIBRARY_NAME} PUBLIC ../external/dlib/include)

# This is a make target, so you can do a "make projBrunner"
add_executable(${APPLICATION_NAME} ${APP_SOURCES})

# Links the target exe (projBrunner) against the library (libprojB) and other required libraries
target_link_libraries(${APPLICATION_NAME} ${LIBRARY_NAME})

message($<TARGET_PROPERTY:${LIBRARY_NAME},INTERFACE_INCLUDE_DIRECTORIES>)

target_include_directories(${APPLICATION_NAME} PRIVATE ../external/dlib/include)
target_include_directories(${APPLICATION_NAME} PRIVATE $<TARGET_PROPERTY:${LIBRARY_NAME},INTERFACE_INCLUDE_DIRECTORIES>)




# expose header files to other libraries
target_include_directories(${LIBRARY_NAME}
    PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/../include/
)

set (LIBRARY_NAME ${LIBRARY_NAME} PARENT_SCOPE) 


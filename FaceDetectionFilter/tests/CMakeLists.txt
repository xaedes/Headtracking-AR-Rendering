set(APPLICATION_NAME_PREFIX "test_${PROJECT_NAME}_")

set(TEST_PROGRAMS
    FaceDetectionFilter
    KalmanFilter
    FirstOrderScalarFilter
)

include_directories ($<TARGET_PROPERTY:catch,INTERFACE_INCLUDE_DIRECTORIES>)

foreach(program_name ${TEST_PROGRAMS})
    set(APPLICATION_NAME "${APPLICATION_NAME_PREFIX}${program_name}_run")
    add_executable(${APPLICATION_NAME} "${program_name}.cpp")
    target_link_libraries(${APPLICATION_NAME} catch ${LIBRARY_NAME} )
endforeach(program_name)


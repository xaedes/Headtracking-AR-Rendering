# Define two variables in order not to repeat ourselves.
set(APPLICATION_NAME "${PROJECT_NAME}_run")


set(APP_SOURCES
	Main.cpp
	Camera.cpp
	Keyboard.cpp
	KeyboardControl.cpp
)

set(QT_SOURCES
	App.cpp
	Mainwindow.cpp
	GLWidget.cpp
)
set(QT_HEADERS
	App.h
	Mainwindow.h
	GLWidget.h
)

set(QT_FILES
	Mainwindow.ui
)

qt5_wrap_cpp(QT_HEADERS_MOC ${QT_HEADERS})
qt5_wrap_ui (QT_FILES_MOC ${QT_FILES})

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
include_directories ($<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>)
include_directories ($<TARGET_PROPERTY:AsusXtionSensor,INTERFACE_INCLUDE_DIRECTORIES>)
include_directories ($<TARGET_PROPERTY:FaceDetectionComponent,INTERFACE_INCLUDE_DIRECTORIES>)

# This is a make target, so you can do a "make projBrunner"
add_executable(
	${APPLICATION_NAME} 
	${APP_SOURCES}
	${QT_SOURCES} 
	${QT_HEADERS_MOC}
	${QT_FILES_MOC} 
)

# Links the target exe (projBrunner) against the library (libprojB) and other required libraries
target_link_libraries(${APPLICATION_NAME} Qt5::Widgets )
target_link_libraries(${APPLICATION_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
target_link_libraries(${APPLICATION_NAME} ${OpenCV_LIBS})
target_link_libraries(${APPLICATION_NAME} common)
target_link_libraries(${APPLICATION_NAME} AsusXtionSensor)
target_link_libraries(${APPLICATION_NAME} FaceDetectionComponent)


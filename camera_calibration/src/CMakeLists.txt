

### ~~~~~~~~~~~~ BUILD RELEVANT LIBRARIES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_library(CalibrationLibrary STATIC camera_calibration.cpp ../include/camera_calibration.h)
target_link_libraries(CalibrationLibrary ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_precompile_headers(CalibrationLibrary PRIVATE ../include/pch.h)

### ~~~~~~~~~~~~ CREATE EXECUTABLE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

set(project_sources main.cpp)
add_executable(${PROJECT_NAME}_run ${project_sources} )
target_link_libraries(${PROJECT_NAME}_run CalibrationLibrary)  



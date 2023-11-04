### ~~~~~~~~~~~~ CREATE EXECUTABLE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(cameracalibration_node 
                main.cpp 
                camera_calibration.cpp)

target_link_libraries(cameracalibration_node 
                        ${OpenCV_LIBS}
                        ${catkin_LIBRARIES})  



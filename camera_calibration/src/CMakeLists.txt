### ~~~~~~~~~~~~ CREATE EXECUTABLE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable(cameracalibration 
                main.cpp 
                camera_calibration.cpp)

target_link_libraries(cameracalibration 
                        ${OpenCV_LIBS}
                        ${catkin_LIBRARIES})  



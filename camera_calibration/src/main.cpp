
#include <camera_calibration.h> 
// #include <ros/ros.h>
#include <iostream>
#include <limits.h>
#include <unistd.h>

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "stereo_camera_calibration");
    
    // for (int i=0; i < argc; i++) std::cout << "argv[i]: "<< argv[i] << std::endl;

    // std::cout << "argc: "<< argc << std::endl;
    std::string arg_str;
    ros::NodeHandle nh("~");

    nh.getParam("/camera_calibration/check_frame_rate", arg_str);
    std::cout << arg_str << std::endl;
    ROS_INFO("Got parameter : %s", arg_str.c_str());

    // char result[ PATH_MAX ];
    // ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
    // std::cout << "\nstd::string( result, (count > 0) ? count : 0 ); " << std::string( result, (count > 0) ? count : 0 );
    // /home/greggz/Projects/catkin_ws/devel/lib/camera_calibration/camera_calibration_run


    StereoCalibration s;
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Checking frame rate");
        s.check_frame_rate(0, cv::CAP_ANY);
        // s.check_frame_rate(1, cv::CAP_ANY);
    }

    nh.getParam("/camera_calibration/record_save", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Capture images for calibration");
        s.capture_stereo_imgs_for_calibration();
    }

    nh.getParam("/camera_calibration/calibrate", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        // s.calibrate_cameras();
        // s.calibrate_stereo();
    }

    nh.getParam("/camera_calibration/test_calibration", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Testing calibration via live-image feed");
        // s.test_calibration();
    }

    // Spin
    // ros::spin ();

    return 0;
}



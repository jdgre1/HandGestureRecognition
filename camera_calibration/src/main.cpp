
#include <camera_calibration.h> 


int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "stereo_camera_calibration");
    
    std::string arg_str;
    ros::NodeHandle nh("~");
    StereoCalibration s;
    
    nh.getParam("/camera_calibration/check_frame_rate", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Checking frame rate");
        s.check_frame_rate(0, cv::CAP_ANY);
        s.check_frame_rate(1, cv::CAP_ANY);
    }

    nh.getParam("/camera_calibration/record_save", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Capture images for calibration");
        s.capture_stereo_imgs_for_calibration();
    }

    nh.getParam("/camera_calibration/calibrate_single_cameras", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        bool save_params = true;
        bool draw_corners = true;
        s.calibrate_single_cameras(save_params, draw_corners);
    }

    nh.getParam("/camera_calibration/calibrate_stereo", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        s.calibrate_stereo();
    }

    nh.getParam("/camera_calibration/test_calibration", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Testing calibration via live-image feed.\nRectifying image and undistorting.");
        s.test_calibration();
    }

    // Spin
    // ros::spin ();

    return 0;
}




#include <camera_calibration.h> 

bool parameter_true(std::string, ros::NodeHandle& nh,std::string& arg_str);
int process_params();

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "stereo_camera_calibration");
    
    process_params();
    // std::string arg_str;
    // ros::NodeHandle nh("~");
    // StereoCalibration s;
    
    // nh.getParam("/camera_calibration/check_frame_rate", arg_str);
    // if(arg_str.compare("true") == 0) 
    // {   
    //     ROS_INFO("Checking frame rate");
    //     s.check_frame_rate(0, cv::CAP_ANY);
    //     s.check_frame_rate(1, cv::CAP_ANY);
    // }

    // nh.getParam("/camera_calibration/record_save", arg_str);
    // if(arg_str.compare("true") == 0) 
    // {   
    //     ROS_INFO("Capture images for calibration");
    //     s.capture_stereo_imgs_for_calibration();
    // }

    // nh.getParam("/camera_calibration/calibrate_single_cameras", arg_str);
    // if(arg_str.compare("true") == 0) 
    // {   
    //     ROS_INFO("Calibrating cameras for stereovision");
    //     bool save_params = true;
    //     bool draw_corners = true;
    //     s.calibrate_single_cameras(save_params, draw_corners);
    // }

    // nh.getParam("/camera_calibration/calibrate_stereo", arg_str);
    // if(arg_str.compare("true") == 0) 
    // {   
    //     ROS_INFO("Calibrating cameras for stereovision");
    //     s.calibrate_stereo();
    // }

    // nh.getParam("/camera_calibration/test_calibration", arg_str);
    // if(arg_str.compare("true") == 0) 
    // {   
    //     ROS_INFO("Testing calibration via live-image feed.\nRectifying image and undistorting.");
    //     s.test_calibration();
    // }

    // nh.getParam("/camera_calibration/run_depth_map", arg_str);
    // if(arg_str.compare("true") == 0) 
    // {   
    //     ROS_INFO("Creating live depth-map.");
    //     s.create_live_depth_map();
    // }
    
    // Spin
    // ros::spin ();

    return 0;
}


int process_params()
{
    std::string arg_str;
    ros::NodeHandle nh("~");
    StereoCalibration s;
    

    // if(strcasecmp(nh.getParam("/camera_calibration/check_frame_rate", "true") == 0)
    if(parameter_true("check_frame_rate", nh, arg_str))
    {   
        ROS_INFO("Checking frame rate");
        s.check_frame_rate(0, cv::CAP_ANY);
        s.check_frame_rate(1, cv::CAP_ANY);
    }


    // if(strcasecmp(nh.getParam("/camera_calibration/record_save", "true") == 0)
    if(parameter_true("record_save", nh, arg_str))
    {
        ROS_INFO("Capture images for calibration");
        s.capture_stereo_imgs_for_calibration();
    }


    // if(strcasecmp(nh.getParam("/camera_calibration/calibrate_single_cameras", "true") == 0)
    if(parameter_true("calibrate_single_cameras", nh, arg_str))
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        bool save_params = true;
        bool draw_corners = true;
        s.calibrate_single_cameras(save_params, draw_corners);
    }


    // if(strcasecmp(nh.getParam("/camera_calibration/calibrate_stereo", "true") == 0)
    if(parameter_true("calibrate_stereo", nh, arg_str))
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        s.calibrate_stereo();
    }


    // if(strcasecmp(nh.getParam("/camera_calibration/test_calibration", "true") == 0)
    if(parameter_true("test_calibration", nh, arg_str))
    {   
        ROS_INFO("Testing calibration via live-image feed.\nRectifying image and undistorting.");
        s.test_calibration();
    }


    // if(strcasecmp(nh.getParam("/camera_calibration/run_depth_map", "true") == 0)
    if(parameter_true("run_depth_map", nh, arg_str))
    {   
        ROS_INFO("Creating live depth-map.");
        s.create_live_depth_map();
    }

}


bool parameter_true(std::string param, ros::NodeHandle& nh, std::string& arg_str) 
{   
    nh.getParam("/camera_calibration/check_frame_rate", arg_str);
    return arg_str == "true";
}
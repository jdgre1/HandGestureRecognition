
#include <camera_calibration.h> 

bool parameterExists(std::string, ros::NodeHandle& nh,std::string& arg_str);
int processParams();

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "stereo_camera_calibration");
    
    processParams();
    std::string arg_str;
    ros::NodeHandle nh("~");
    StereoCalibration s;
    
    nh.getParam("/camera_calibration/check_frame_rate", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Checking frame rate");
        s.checkFrameRate(0, cv::CAP_ANY);
        s.checkFrameRate(1, cv::CAP_ANY);
    }

    nh.getParam("/camera_calibration/record_save", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Capture images for calibration");
        s.captureStereoImgsForCalibration();
    }

    nh.getParam("/camera_calibration/calibrate_single_cameras", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        bool save_params = true;
        bool draw_corners = true;
        s.calibrateSingleCameras(save_params, draw_corners);
    }

    nh.getParam("/camera_calibration/calibrate_stereo", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        s.calibrateStereo();
    }

    nh.getParam("/camera_calibration/test_calibration", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Testing calibration via live-image feed.\nRectifying image and undistorting.");
        s.testCalibration();
    }

    nh.getParam("/camera_calibration/run_depth_map", arg_str);
    if(arg_str.compare("true") == 0) 
    {   
        ROS_INFO("Creating live depth-map.");
        s.createLiveDepthMap();
    }
    
    ros::spin ();

    return 0;
}


int processParams()
{
    std::string arg_str;
    ros::NodeHandle nh("~");
    StereoCalibration s;
    
    if(parameterExists("check_frame_rate", nh, arg_str))
    {   
        ROS_INFO("Checking frame rate");
        s.checkFrameRate(0, cv::CAP_ANY);
        s.checkFrameRate(1, cv::CAP_ANY);
    }

    if(parameterExists("record_save", nh, arg_str))
    {
        ROS_INFO("Capture images for calibration");
        s.captureStereoImgsForCalibration();
    }

    if(parameterExists("calibrate_single_cameras", nh, arg_str))
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        bool save_params = true;
        bool draw_corners = true;
        s.calibrateSingleCameras(save_params, draw_corners);
    }

    if(parameterExists("calibrate_stereo", nh, arg_str))
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        s.calibrateStereo();
    }

    if(parameterExists("test_calibration", nh, arg_str))
    {   
        ROS_INFO("Testing calibration via live-image feed.\nRectifying image and undistorting.");
        s.testCalibration();
    }

    if(parameterExists("run_depth_map", nh, arg_str))
    {   
        ROS_INFO("Creating live depth-map.");
        s.createLiveDepthMap();
    }
    return 0;

}


bool parameterExists(std::string param, ros::NodeHandle& nh, std::string& arg_str) 
{   
    nh.getParam("/camera_calibration/"  + param, arg_str);
    return arg_str == "true";
}
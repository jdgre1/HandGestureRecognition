
#include <camera_calibration.h> 


int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "stereo_camera_calibration");
    ros::NodeHandle nh("~");
    
    StereoCalibration s(nh);
    s.run();

    return 0;
}
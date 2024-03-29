#pragma once
#ifndef CAMERA_CALIBRATION__H__
#define CAMERA_CALIBRATION__H__
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/cudafilters.hpp>
#include <opencv2/core/utils/trace.hpp>
#include <opencv2/opencv.hpp>

#define HAS_CUDA 1
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
static const cv::Size IMAGE_SIZE(IMG_WIDTH, IMG_HEIGHT);

class StereoCalibration
{
public:
    

    // Delete copy constructor
    StereoCalibration(StereoCalibration& other) = delete;

    // Delete assignment operator
    void operator=(const StereoCalibration &) = delete;

    static StereoCalibration *GetInstance(ros::NodeHandle& nh);

    void run();

    struct StereoParams {
        // Checkerboard size
        //
        int CHECKERBOARD[2]{7, 10};

        // Files to store parameters
        std::string pre_stereo_calib_file;

        // Single vector of 3D points;
        std::vector<cv::Point3f> objp;

        // Creating vector to store vectors of 3D points for each checkerboard image
        std::vector<std::vector<cv::Point3f>> objpoints;

        // Creating vector to store vectors of 2D points for each checkerboard image
        std::vector<std::vector<cv::Vec2f>> imgpoints_left, imgpoints_right;
        std::vector<std::vector<cv::Vec3f>> objpoints_left, objpoints_right;

        // Image size for single-camera calibration method
        int img_size_rows, img_size_cols;
        cv::Size img_size;

        int num_successful_calib_imgs;

        // The overall RMS re-projection error.
        float reproj_error;

        // Individual camera calibration parameters
        cv::Mat mtxL, distL, R_L, T_L;
        cv::Mat mtxR, distR, R_R, T_R;
        cv::Mat new_mtxL, new_mtxR, Rot, Trns;
        cv::Mat left_stereo_map1, left_stereo_map2, right_stereo_map1, right_stereo_map2;
    };

    struct DepthmapParams {
        const int mindisparity_limit = 100;
        const int numdisparities_max = 100;
        const int blocksize_max = 11;
        const int disp12maxdiff_max = 2;
        const int uniquenessratio_max = 30;
        const int specklewindowsize_max = 250;
        const int specklerange_max = 20;

        int alpha_slider;
        double alpha;
        double beta;

        /*int minDisparity = 20;
        int numDisparities = 15;
        int blockSize = 1;
        int disp12MaxDiff = 1;
        int uniquenessRatio = 30;
        int speckleWindowSize = 36;
        int speckleRange = 7;*/

        int min_disparity = 97;
        int num_disparities = 54;
        int blocksize = 11;
        int disp12_max_diff = 1;
        int uniqueness_ratio = 30;
        int speckle_window_size = 37;
        int speckle_range = 20;

        int prev_min_disparity = 0;
        int prev_num_disparities = 64;
        int prev_blocksize = 8;
        int prev_disp12_max_diff = 1;
        int prev_uniqueness_ratio = 10;
        int prev_speckle_window_size = 10;
        int prev_speckle_range = 8;

        cv::Mat left_after_rectify, right_after_rectify, dst, orig_frame0, orig_frame1;
        cv::Mat left_stereo_map1, left_stereo_map2;
        cv::Mat right_stereo_map1, right_stereo_map2;

        cv::String window_name = "Depth Map";
        cv::String min_disparity_title = "min disparity";
        cv::String num_disparities_title = "num disparities";
        cv::String block_size_title = "block size";
        cv::String disp12_max_diff_title = "disp12MaxDiff";
        cv::String uniqueness_ratio_title = "uniqueness ratio";
        cv::String speckle_window_size_title = "speckleWindowSize";
        cv::String speckle_range_title = "speckleRange";

        cv::Ptr<cv::StereoSGBM> stereo;
        float focal_length;
        float baseline;
        float scaled_base_times_focal_length;
        cv::Point2f hand_center;

    }; // depthmap_data;

private:
    // Constructor
    StereoCalibration(ros::NodeHandle& nh);

    bool runParamServerConfig(void);
    bool initialiseCameras();
    int checkFrameRate(int deviceID, int apiID);
    void captureStereoImgsForCalibration();
	void processCheckerboardCorners(bool draw_corners);
    bool checkCheckerboardCornersExist(cv::Mat& frame0, cv::Mat& frame1);
	
	void calibrateSingleCameras(bool save_params, bool draw_corners);
    void calibrateStereoCamera();
	void saveCameraCalibParams();
    void loadCameraCalibParams();
    
    void rectifyAndUndistort();
    void testCalibration();
    void createLiveDepthMap();
    void createTrackbars();
    static void onTrackbar(int, void*);

    int m_deviceID;
    int m_apiID = cv::CAP_ANY; // 0 = autodetect default API,	1 = open other camera

    // Frame recording
    std::vector<cv::Mat> m_frames;
    std::string m_framesSaveDir;

    // Calibration parameters
    int m_numCalibrationImgs = 50;

    // New frame every 2 seconds taken (Chance to move checkerboard to new position)
    float m_frameRate = 0.5;

    // Stereo-calibration occurs at same time
    cv::VideoCapture m_cap0;
    cv::VideoCapture m_cap1;
    int m_deviceID0 = 0;
    int m_deviceID1 = 2;
    int m_apiID0 = cv::CAP_ANY; // 0 = autodetect default API,	1 = open other camera
    int m_apiID1 = cv::CAP_ANY; // 0 = autodetect default API,	1 = open other camer
    bool m_reloadParams = false;
    bool m_avoidSavingAndLoading = true;

    StereoParams m_stereoParams;
    DepthmapParams m_depthmapData;

    // Test calibration by undistorting images
    int m_numImgsToTest = 100;
	ros::NodeHandle& m_nh;

    static StereoCalibration* instance;
};

#endif



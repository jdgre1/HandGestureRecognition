#pragma once
#ifndef CAMERA_CALIBRATION__H__
#define CAMERA_CALIBRATION__H__
#include <pch.h>
#include <fstream>

#define HAS_CUDA 1
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
static const cv::Size IMAGE_SIZE(IMG_WIDTH, IMG_HEIGHT);


class StereoCalibration 
{
	
	public:
		// Constructor
		StereoCalibration(const int&& device_id);
		StereoCalibration();

		// Destructor
		~StereoCalibration();

		int check_frame_rate(int deviceID, int apiID);
		void calibrate_single_cameras(bool save_params, bool draw_corners);
		void capture_stereo_imgs_for_calibration();
		void calibrate_stereo();
		bool verify_checkerboard_corners(cv::Mat& frame0, cv::Mat& frame1); 
		bool initialise_cameras(); 
		void save_camera_calib_params();
		void load_camera_calib_params();
		void process_checkerboard_corners(bool draw_corners);
		void test_calibration();

		struct StereoParams 
		{
			// Checkerboard size
			// 
    		int CHECKERBOARD[2]{ 7,10 };

			// Files to store parameters
			std::string pre_stereo_calib_file;
			
			// Single vector of 3D points;
			std::vector<cv::Point3f> objp;

			// Creating vector to store vectors of 3D points for each checkerboard image
			std::vector<std::vector<cv::Point3f> > objpoints;

			// Creating vector to store vectors of 2D points for each checkerboard image
			std::vector<std::vector<cv::Vec2f> > imgpointsLeft, imgpointsRight;  
			std::vector<std::vector<cv::Vec3f> > objpointsLeft, objpointsRight;

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
		};
	
	private:

		int m_deviceID;						
		int m_apiID = cv::CAP_ANY;		// 0 = autodetect default API,	1 = open other camera

		// Frame recording
		std::vector<cv::Mat> m_frames;
		std::string m_frames_save_dir;

		// Calibration parameters
		int m_num_calibration_imgs = 50;

		// New frame every 2 seconds taken (Chance to move checkerboard to new position)
		float m_frame_rate = 0.5; 

		// Stereo-calibration occurs at same time
		cv::VideoCapture m_cap0;
		cv::VideoCapture m_cap1;
		int m_deviceID0 = 0;
		int m_deviceID1 = 2;
		int m_apiID0 = cv::CAP_ANY;		// 0 = autodetect default API,	1 = open other camera
		int m_apiID1 = cv::CAP_ANY;		// 0 = autodetect default API,	1 = open other camer
		StereoParams m_stereo_params;
		bool m_reload_params = false;
		bool m_avoid_saving_and_loading = true;
		
		// Test calibration by undistorting images
		int m_num_imgs_to_test = 100;

};


#endif
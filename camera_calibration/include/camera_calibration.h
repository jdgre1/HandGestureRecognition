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

		int checkFrameRate(int deviceID, int apiID);
		void calibrateSingleCameras(bool save_params, bool draw_corners);
		void captureStereoImgsForCalibration();
		void calibrateStereo();
		bool verifyCheckerboardCorners(cv::Mat& frame0, cv::Mat& frame1); 
		bool initialiseCameras(); 
		void saveCameraCalibParams();
		void loadCameraCalibParams();
		void processCheckerboardCorners(bool draw_corners);
		void rectifyAndUndistort();
		void testCalibration();
		void createLiveDepthMap();
		void createTrackbars();
		static void onTrackbar(int, void*);

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
			std::vector<std::vector<cv::Vec2f> > imgpoints_left, imgpoints_right;  
			std::vector<std::vector<cv::Vec3f> > objpoints_left, objpoints_right;

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

		struct DepthmapParams
		{
			const int mindisparity_max = 100;
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

			int minDisparity = 97;
			int numDisparities = 54;
			int blockSize = 11;
			int disp12MaxDiff = 1;
			int uniquenessRatio = 30;
			int speckle_window_size = 37;
			int speckle_range = 20;

			int prev_min_disparity = 0;
			int prev_num_disparities = 64;
			int prev_blocksize = 8;
			int prev_disp12_max_diff = 1;
			int prev_uniqueness_ratio = 10;
			int prev_speckle_window_size = 10;
			int prev_speckle_range = 8;

			cv::Mat left_nice, right_nice, dst, orig_frame0, orig_frame1;
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
		bool m_reloadParams = false;
		bool m_avoidSavingAndLoading = true;

		StereoParams m_stereoParams;
		DepthmapParams m_depthmapData;
		
		// Test calibration by undistorting images
		int m_num_imgs_to_test = 100;

};


#endif
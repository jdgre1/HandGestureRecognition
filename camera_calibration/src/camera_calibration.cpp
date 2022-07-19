#include <camera_calibration.h> 

StereoCalibration::StereoCalibration() 
{
    m_frames_save_dir = ros::package::getPath("camera_calibration") + "/data/";
    m_stereo_params.pre_stereo_calib_file = ros::package::getPath("camera_calibration") + "/config/pre_stereo_calib_file.xml";
};

StereoCalibration::~StereoCalibration() {}

int StereoCalibration::check_frame_rate(int deviceID, int apiID) 
{
    // open selected camera using selected API
	cv::VideoCapture v_cap0;
	cv::VideoCapture v_cap1;
    v_cap0.open(deviceID, apiID);
    // const char *pipeline = " tcambin serial=15810833 ! video/x-raw, format=BGRx, width=1280,height=960, framerate=25/1 ! videoconvert ! appsink";
    // v_cap1.open(2, cv::CAP_GSTREAMER);
    v_cap1.open(2, apiID);

    // v_cap1.open(1, cv::CAP_V4L);

    // check if we succeeded
    if (!v_cap0.isOpened() && !v_cap1.isOpened()) {
        std::cerr << "ERROR! Unable to open cameras \n";
        v_cap0.release();
        v_cap1.release();
        return -1;
    }


    // With webcam get(CV_CAP_PROP_FPS) does not work.
    // Let's see for ourselves.
    // double fps = video.get(CV_CAP_PROP_FPS);
    // If you do not care about backward compatibility
    // You can use the following instead for OpenCV 3
    double fps = v_cap0.get(cv::CAP_PROP_FPS);
    std::cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << std::endl;

    // Start and end times
    time_t start, end;

    // Variable for storing video frames
    cv::Mat frame;
    std::cout << "Capturing " << m_num_imgs_to_test << " frames" << std::endl;

    // Start time
    time(&start);

    // Grab a few frames
    for (int i = 0; i < m_num_imgs_to_test; i++) {
        v_cap0 >> frame;
    }

    // End Time
    time(&end);

    // Time elapsed
    double seconds = difftime(end, start);
    std::cout << "Time taken : " << seconds << " seconds" << std::endl;

    // Calculate frames per second
    fps = m_num_imgs_to_test / seconds;
    std::cout << "Estimated frames per second : " << fps << std::endl;

    // Release video
    v_cap0.release();
    return 0;
}


bool StereoCalibration::verify_checkerboard_corners(cv::Mat& frame0, cv::Mat& frame1) 
{
    /* Finding checker board corners: If desired number of corners are found in the image then success = true  */
    
    // Defining the dimensions of checkerboard
    int CHECKERBOARD[2]{ 7,10 };

    if (m_stereo_params.objp.size() == 0) 
        for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
            for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
                m_stereo_params.objp.push_back(cv::Point3f(j, i, 0));

    cv::Mat gray0, gray1;
    std::vector<cv::Point2f> corner_pts0, corner_pts1;
    cv::cvtColor(frame0, gray0, cv::COLOR_BGR2GRAY);
    bool success = cv::findChessboardCorners(gray0, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts0, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
    std::cout << "Detected corners frame0? " << success << std::endl;
    cv::imshow("frame0", frame0);
    cv::imshow("frame1", frame1);
    cv::waitKey(1000);
    
    // If first frame successful, try other camera too:
    if (success) {  
        cv::cvtColor(frame1, gray1, cv::COLOR_BGR2GRAY);
        success = cv::findChessboardCorners(gray1, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts1, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        std::cout << "Detected corners frame1? " << success << std::endl;
    }
    
    // If both successful, save points, draw checkboard corners and display
    if (success) {
        cv::Mat frame0_copy = frame0.clone();
        cv::Mat frame1_copy = frame1.clone();
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);

        cv::cornerSubPix(gray0, corner_pts0, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        cv::cornerSubPix(gray1, corner_pts1, cv::Size(11, 11), cv::Size(-1, -1), criteria);

        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame0_copy, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts0, success);
        cv::drawChessboardCorners(frame1_copy, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts1, success);

        // Display error between images on frames: Helps determine if images should be included for calibration:

        cv::imshow("frame0", frame0_copy);
        cv::imshow("frame1", frame1_copy);
        
        // Allow manual-user check:
        int key = cv::waitKey();
        switch (key)
        {
        case ((int)('y')):
            // Keep this frame:
            return success;

        case ((int)('n')):
            // Reject this frame;
            return false;
        }
       
    }
    return false;
}


bool StereoCalibration::initialise_cameras() 
{
    // Open both cameras:
    m_cap0.open(m_deviceID0, cv::CAP_ANY);
    m_cap1.open(m_deviceID1, cv::CAP_ANY);
    // m_cap1.open(m_deviceID1, cv::CAP_V4L2);


    // check if we succeeded
    if (!m_cap0.isOpened() || !m_cap1.isOpened()) {
        std::cerr << "ERROR! Unable to open one or more cameras\n";
        return false;
    }

    //--- GRAB AND WRITE LOOP

    // Begin capturing frames
    std::cout << "Capturing " << m_num_calibration_imgs << " frames for camera " + std::to_string(m_deviceID) << std::endl;

    return true;
}


void StereoCalibration::capture_stereo_imgs_for_calibration() 
{
    // temp variables
    cv::Mat frame0, frame1;
    bool frame_error;
    int frame_count;

    // loop time between consecutive frames
    time_t curr_time, prev;

    // start cameras
    if (initialise_cameras()) 
    {
        // Start time
        time(&curr_time);
        time(&prev);

        // Incremental counter of successful frames
        frame_count = 0;
        // Set 1 frame every 2 seconds as the frame rate - (enough time to change position of chessboard)

        while (frame_count < m_num_calibration_imgs) 
        {
            m_cap0 >> frame0;
            m_cap1 >> frame1;
            // check if we succeeded
            if (frame0.empty() || frame1.empty()) {
                std::cerr << "ERROR! blank frame of cam0 or cam1 grabbed\n";
                continue;
            }

            time(&curr_time);
            if ((curr_time - prev >= 1. / m_frame_rate) && verify_checkerboard_corners(frame0, frame1)) {
                frame_count++;
                prev = curr_time;
                std::string frame_num_str = std::to_string(frame_count);
                cv::imwrite(m_frames_save_dir + "0/frame0_" + std::to_string(frame_count) + ".png", frame0);
                cv::imwrite(m_frames_save_dir + "1/frame1_" + std::to_string(frame_count) + ".png", frame1);
                std::cout << "frame_count: " << frame_count << std::endl;
            }
            
        }
    }
    return;
}


void StereoCalibration::process_checkerboard_corners(bool draw_corners)
{
      // Image Data
    int total_photos = 50;
    cv::Mat imgL, imgR, grayL, grayR;
   
    // Checkerboard settings
    cv::Vec3f pt(0, 0, 0);
    std::vector<cv::Vec3f> objp;    
    cv::Size CHECKERBOARD(7, 10);
    for (int i = 0; i < CHECKERBOARD.height; i++) for (int j = 0; j < CHECKERBOARD.width; j++) objp.push_back(cv::Vec3f(j, i, 0));

    if (draw_corners)
        fprintf(stderr, "You can press 'Q' to quit this script.\n");

    for (int photo_counter = 1; photo_counter <= m_num_calibration_imgs; photo_counter++)
    {
        std::cout << "imgL: " << m_frames_save_dir + "0/frame0_" + std::to_string(photo_counter) + ".png" << std::endl;
        std::cout << "imgR: " << m_frames_save_dir + "1/frame1_" + std::to_string(photo_counter) + ".png" << std::endl;
        fprintf(stderr, "Import pair No %d\n", photo_counter);
        imgL = cv::imread(m_frames_save_dir + "0/frame0_" + std::to_string(photo_counter) + ".png");
        imgR = cv::imread(m_frames_save_dir + "1/frame1_" + std::to_string(photo_counter) + ".png");

        if (imgR.empty() || imgL.empty())
        {
            fprintf(stderr, "There are no images in pair No %d\n", photo_counter);
            continue;
        }

        // If stereopair is complete - go to processing
        cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);

        // Find the chessboard corners
        std::vector<cv::Vec2f> cornersL, cornersR;
        bool retL = cv::findChessboardCorners(grayL, CHECKERBOARD, cornersL, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool retR = cv::findChessboardCorners(grayR, CHECKERBOARD, cornersR, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);

        // Draw images with corners found
        if (draw_corners)
        {
            cv::drawChessboardCorners(imgL, CHECKERBOARD, cornersL, retL);
            cv::imshow("Corners LEFT", imgL);
            cv::drawChessboardCorners(imgR, CHECKERBOARD, cornersR, retR);
            cv::imshow("Corners RIGHT", imgR);
            char key = cv::waitKey();
            if (key == 'q' || key == 'Q')
                exit(1);
        }

        // Refine corners and add to array for processing
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);
        if (retL && retR)
        {
            m_stereo_params.objpointsLeft.push_back(objp);
            cv::cornerSubPix(grayL, cornersL, cv::Size(3, 3), cv::Size(-1, -1), criteria);
            m_stereo_params.imgpointsLeft.push_back(cornersL);
            m_stereo_params.objpointsRight.push_back(objp);
            cv::cornerSubPix(grayR, cornersR, cv::Size(3, 3), cv::Size(-1, -1), criteria);
            m_stereo_params.imgpointsRight.push_back(cornersR);
        }
        else
        {
            fprintf(stderr, "Pair No %d ignored, as no chessboard found\n", photo_counter);
            continue;
        }
    }

    return;
}


void StereoCalibration::calibrate_single_cameras(bool save_params, bool draw_corners)
{
    
    process_checkerboard_corners(draw_corners);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Single left camera calibration: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    fprintf(stderr, "Processing left camera calibration...\n");
    cv::Mat rvecs, tvecs;
    double rms = cv::calibrateCamera(m_stereo_params.objpointsLeft, m_stereo_params.imgpointsLeft, IMAGE_SIZE, m_stereo_params.mtxL, m_stereo_params.distL, rvecs, tvecs);
    std::cout << "Left camera calibration rms of " << rms << std::endl;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Single right camera calibration: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    fprintf(stderr, "Processing right camera calibration...\n");
    rms = cv::calibrateCamera(m_stereo_params.objpointsRight, m_stereo_params.imgpointsRight, IMAGE_SIZE, m_stereo_params.mtxR, m_stereo_params.distR, rvecs, tvecs);
    std::cout << "Right camera calibration rms of " << rms << std::endl;

    if(save_params) save_camera_calib_params();

    return;
}


void StereoCalibration::calibrate_stereo()
{
    load_camera_calib_params(); 
    
    m_stereo_params.new_mtxL = cv::getOptimalNewCameraMatrix(m_stereo_params.mtxL, m_stereo_params.distL, IMAGE_SIZE, 1, IMAGE_SIZE, 0);
    m_stereo_params.new_mtxR = cv::getOptimalNewCameraMatrix(m_stereo_params.mtxR, m_stereo_params.distR, IMAGE_SIZE, 1, IMAGE_SIZE, 0);

    cv::Mat Emat, Fmat;
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    double stereo_rms = cv::stereoCalibrate(m_stereo_params.objpointsRight, m_stereo_params.imgpointsLeft, m_stereo_params.imgpointsRight,  m_stereo_params.new_mtxL, 
                                                m_stereo_params.distL, m_stereo_params.new_mtxR, m_stereo_params.distR, IMAGE_SIZE, m_stereo_params.Rot, m_stereo_params.Trns, Emat, Fmat, flag,
                                                cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));
    std::cout << "stereo_rms: " << stereo_rms << std::endl;
    return;
}

// Save the parameters obtained from successful calibration:
void StereoCalibration::save_camera_calib_params() 
{
    cv::FileStorage fs(m_stereo_params.pre_stereo_calib_file, cv::FileStorage::WRITE);
    fs << "imgpointsL" << m_stereo_params.imgpointsLeft;
    fs << "imgpointsR" << m_stereo_params.imgpointsRight;
    fs << "objpointsRight" << m_stereo_params.objpointsRight;
    fs << "distL" << m_stereo_params.distL;
    fs << "distR" << m_stereo_params.distR;
    fs << "mtxL" << m_stereo_params.mtxL;
    fs << "mtxR" << m_stereo_params.mtxR;
    return;
}


void StereoCalibration::load_camera_calib_params() 
{
    cv::FileStorage fs(m_stereo_params.pre_stereo_calib_file, cv::FileStorage::READ);
    fs["imgpointsL"] >> m_stereo_params.imgpointsLeft;
    fs["imgpointsR"] >> m_stereo_params.imgpointsRight;
    fs["objpointsRight"] >> m_stereo_params.objpointsRight;
    fs["distL"] >> m_stereo_params.distL;
    fs["distR"] >> m_stereo_params.distR;
    fs["mtxL"] >> m_stereo_params.mtxL;
    fs["mtxR"] >> m_stereo_params.mtxR;
    return;
}


void StereoCalibration::test_calibration()
{   
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Load Test Images: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    cv::Mat img_left = cv::imread(m_frames_save_dir + "0/frame0_1.png");
    cv::Mat img_right = cv::imread(m_frames_save_dir + "1/frame1_1.png");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Calibration: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    calibrate_stereo();
   
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Rectification: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
    cv::stereoRectify( m_stereo_params.new_mtxL, m_stereo_params.distL, m_stereo_params.new_mtxR, m_stereo_params.distR, IMAGE_SIZE, m_stereo_params.Rot, 
                        m_stereo_params.Trns, rect_l, rect_r, proj_mat_l, proj_mat_r, Q, 1);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Undistort: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cv::Mat left_stereo_map1, left_stereo_map2, right_stereo_map1, right_stereo_map2;
    cv::initUndistortRectifyMap( m_stereo_params.new_mtxL, m_stereo_params.distL, rect_l, proj_mat_l, IMAGE_SIZE, CV_16SC2, left_stereo_map1, left_stereo_map2);
    cv::initUndistortRectifyMap(m_stereo_params.new_mtxR, m_stereo_params.distR, rect_r, proj_mat_r, IMAGE_SIZE, CV_16SC2, right_stereo_map1, right_stereo_map2);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Remap: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cv::Mat left_nice, right_nice;
    cv::remap(img_left, left_nice, left_stereo_map1, left_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    cv::remap(img_right, right_nice, right_stereo_map1, right_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    cv::imshow("Left image before rectification", img_left);
    cv::imshow("Right image before rectification", img_right);

    cv::imshow("Left image after rectification", left_nice);
    cv::imshow("Right image after rectification", right_nice);

    cv::waitKey();
    ROS_INFO("Test complete!\n");

}


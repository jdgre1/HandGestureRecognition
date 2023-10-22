#include <camera_calibration.h> 

StereoCalibration::StereoCalibration() 
{
    m_frames_save_dir = ros::package::getPath("camera_calibration") + "/data/";
    m_stereoParams.pre_stereo_calib_file = ros::package::getPath("camera_calibration") + "/config/pre_stereo_calib_file.xml";
};

StereoCalibration::~StereoCalibration() {}

int StereoCalibration::checkFrameRate(int deviceID, int apiID) 
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


bool StereoCalibration::verifyCheckerboardCorners(cv::Mat& frame0, cv::Mat& frame1) 
{
    /* Finding checker board corners: If desired number of corners are found in the image then success = true  */
    
    // Defining the dimensions of checkerboard
    int CHECKERBOARD[2]{ 7,10 };

    if (m_stereoParams.objp.size() == 0) 
        for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
            for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
                m_stereoParams.objp.push_back(cv::Point3f(j, i, 0));

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


bool StereoCalibration::initialiseCameras() 
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


void StereoCalibration::captureStereoImgsForCalibration() 
{
    // temp variables
    cv::Mat frame0, frame1;
    bool frame_error;
    int frame_count;

    // loop time between consecutive frames
    time_t curr_time, prev;

    // start cameras
    if (initialiseCameras()) 
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
            if ((curr_time - prev >= 1. / m_frame_rate) && verifyCheckerboardCorners(frame0, frame1)) {
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


void StereoCalibration::processCheckerboardCorners(bool draw_corners)
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
            m_stereoParams.objpoints_left.push_back(objp);
            cv::cornerSubPix(grayL, cornersL, cv::Size(3, 3), cv::Size(-1, -1), criteria);
            m_stereoParams.imgpoints_left.push_back(cornersL);
            m_stereoParams.objpoints_right.push_back(objp);
            cv::cornerSubPix(grayR, cornersR, cv::Size(3, 3), cv::Size(-1, -1), criteria);
            m_stereoParams.imgpoints_right.push_back(cornersR);
        }
        else
        {
            fprintf(stderr, "Pair No %d ignored, as no chessboard found\n", photo_counter);
            continue;
        }
    }

    return;
}


void StereoCalibration::calibrateSingleCameras(bool save_params, bool draw_corners)
{
    
    processCheckerboardCorners(draw_corners);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Single left camera calibration: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    fprintf(stderr, "Processing left camera calibration...\n");
    cv::Mat rvecs, tvecs;
    double rms = cv::calibrateCamera(m_stereoParams.objpoints_left, m_stereoParams.imgpoints_left, IMAGE_SIZE, m_stereoParams.mtxL, m_stereoParams.distL, rvecs, tvecs);
    std::cout << "Left camera calibration rms of " << rms << std::endl;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Single right camera calibration: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    fprintf(stderr, "Processing right camera calibration...\n");
    rms = cv::calibrateCamera(m_stereoParams.objpoints_right, m_stereoParams.imgpoints_right, IMAGE_SIZE, m_stereoParams.mtxR, m_stereoParams.distR, rvecs, tvecs);
    std::cout << "Right camera calibration rms of " << rms << std::endl;

    if(save_params) saveCameraCalibParams();

    return;
}


void StereoCalibration::calibrateStereo()
{
    loadCameraCalibParams(); 
    
    m_stereoParams.new_mtxL = cv::getOptimalNewCameraMatrix(m_stereoParams.mtxL, m_stereoParams.distL, IMAGE_SIZE, 1, IMAGE_SIZE, 0);
    m_stereoParams.new_mtxR = cv::getOptimalNewCameraMatrix(m_stereoParams.mtxR, m_stereoParams.distR, IMAGE_SIZE, 1, IMAGE_SIZE, 0);

    cv::Mat Emat, Fmat;
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    double stereo_rms = cv::stereoCalibrate(m_stereoParams.objpoints_right, m_stereoParams.imgpoints_left, m_stereoParams.imgpoints_right,  m_stereoParams.new_mtxL, 
                                                m_stereoParams.distL, m_stereoParams.new_mtxR, m_stereoParams.distR, IMAGE_SIZE, m_stereoParams.Rot, m_stereoParams.Trns, Emat, Fmat, flag,
                                                cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));
    std::cout << "stereo_rms: " << stereo_rms << std::endl;
    return;
}

// Save the parameters obtained from successful calibration:
void StereoCalibration::saveCameraCalibParams() 
{
    cv::FileStorage fs(m_stereoParams.pre_stereo_calib_file, cv::FileStorage::WRITE);
    fs << "imgpointsL" << m_stereoParams.imgpoints_left;
    fs << "imgpointsR" << m_stereoParams.imgpoints_right;
    fs << "objpoints_right" << m_stereoParams.objpoints_right;
    fs << "distL" << m_stereoParams.distL;
    fs << "distR" << m_stereoParams.distR;
    fs << "mtxL" << m_stereoParams.mtxL;
    fs << "mtxR" << m_stereoParams.mtxR;
    return;
}


void StereoCalibration::loadCameraCalibParams() 
{
    cv::FileStorage fs(m_stereoParams.pre_stereo_calib_file, cv::FileStorage::READ);
    fs["imgpointsL"] >> m_stereoParams.imgpoints_left;
    fs["imgpointsR"] >> m_stereoParams.imgpoints_right;
    fs["objpoints_right"] >> m_stereoParams.objpoints_right;
    fs["distL"] >> m_stereoParams.distL;
    fs["distR"] >> m_stereoParams.distR;
    fs["mtxL"] >> m_stereoParams.mtxL;
    fs["mtxR"] >> m_stereoParams.mtxR;
    return;
}

void StereoCalibration::rectifyAndUndistort()
{
     // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Rectification: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
    cv::stereoRectify( m_stereoParams.new_mtxL, m_stereoParams.distL, m_stereoParams.new_mtxR, m_stereoParams.distR, IMAGE_SIZE, m_stereoParams.Rot, 
                        m_stereoParams.Trns, rect_l, rect_r, proj_mat_l, proj_mat_r, Q, 1);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Undistort: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cv::initUndistortRectifyMap( m_stereoParams.new_mtxL, m_stereoParams.distL, rect_l, proj_mat_l, IMAGE_SIZE, CV_16SC2, 
                                    m_stereoParams.left_stereo_map1, m_stereoParams.left_stereo_map2);
    cv::initUndistortRectifyMap(m_stereoParams.new_mtxR, m_stereoParams.distR, rect_r, proj_mat_r, IMAGE_SIZE, CV_16SC2, 
                                    m_stereoParams.right_stereo_map1, m_stereoParams.right_stereo_map2);

}

void StereoCalibration::testCalibration()
{   
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Load Test Images: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    cv::Mat img_left = cv::imread(m_frames_save_dir + "0/frame0_1.png");
    cv::Mat img_right = cv::imread(m_frames_save_dir + "1/frame1_1.png");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Calibration: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    calibrateStereo();
   
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Rectification / Undistortion: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    rectifyAndUndistort();

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Remap: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   
    cv::Mat left_nice, right_nice;
    cv::remap(img_left, left_nice, m_stereoParams.left_stereo_map1, m_stereoParams.left_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
    cv::remap(img_right, right_nice, m_stereoParams.right_stereo_map1, m_stereoParams.right_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    cv::imshow("Left image before rectification", img_left);
    cv::imshow("Right image before rectification", img_right);

    cv::imshow("Left image after rectification", left_nice);
    cv::imshow("Right image after rectification", right_nice);

    cv::waitKey();
    ROS_INFO("Test complete!\n");

}

void StereoCalibration::createTrackbars()
{
    cv::namedWindow(m_depthmapData.window_name, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar(m_depthmapData.min_disparity_title, m_depthmapData.window_name, &m_depthmapData.min_disparity, m_depthmapData.mindisparity_limit, onTrackbar);
    cv::createTrackbar(m_depthmapData.num_disparities_title, m_depthmapData.window_name, &m_depthmapData.num_disparities, m_depthmapData.numdisparities_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.block_size_title, m_depthmapData.window_name, &m_depthmapData.blocksize, m_depthmapData.blocksize_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.disp12_max_diff_title, m_depthmapData.window_name, &m_depthmapData.disp12_max_diff, m_depthmapData.disp12maxdiff_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.uniqueness_ratio_title, m_depthmapData.window_name, &m_depthmapData.uniqueness_ratio, m_depthmapData.uniquenessratio_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.speckle_window_size_title, m_depthmapData.window_name, &m_depthmapData.prev_speckle_window_size, m_depthmapData.prev_speckle_window_size, onTrackbar);
    cv::createTrackbar(m_depthmapData.speckle_range_title, m_depthmapData.window_name, &m_depthmapData.speckle_range, m_depthmapData.specklerange_max, onTrackbar);
}

void StereoCalibration::createLiveDepthMap()
{
    calibrateStereo();
    rectifyAndUndistort();
    
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo SGBM Initialisation: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    m_depthmapData.stereo = cv::StereoSGBM::create(m_depthmapData.min_disparity, m_depthmapData.num_disparities, m_depthmapData.blocksize, m_depthmapData.disp12_max_diff,
        m_depthmapData.uniqueness_ratio, m_depthmapData.prev_speckle_window_size, m_depthmapData.speckle_range);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Open Cameras and Read Images: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    bool frame_error;
    bool set_depth_params = true;
    cv::Mat disp;

    if (!initialiseCameras()) return;

    while (1)
    {
        m_cap0 >> m_depthmapData.orig_frame0;
        m_cap1 >> m_depthmapData.orig_frame1;
        frame_error = false;
        // check if we succeeded
        if (m_depthmapData.orig_frame0.empty() || m_depthmapData.orig_frame1.empty()) {
            std::cerr << "ERROR! blank frame of cam0 or cam1 grabbed\n";
            frame_error = true;
        }

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Remap: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        cv::remap(m_depthmapData.orig_frame0, m_depthmapData.left_nice, m_depthmapData.left_stereo_map1, m_depthmapData.left_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
        cv::remap(m_depthmapData.orig_frame1, m_depthmapData.right_nice, m_depthmapData.right_stereo_map1, m_depthmapData.right_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        cv::imshow("Left image before rectification", m_depthmapData.orig_frame0);
        cv::imshow("Right image before rectification", m_depthmapData.orig_frame1);

        cv::imshow("Left image after rectification", m_depthmapData.left_nice);
        cv::imshow("Right image after rectification", m_depthmapData.right_nice);


        if (set_depth_params)
        {
            createTrackbars();

            int key = cv::waitKey();

            switch (key)
            {
            case ((int)('y')):
                cv::waitKey(1000);
                cv::destroyAllWindows();
                std::cout << "\nParameters have been set." << std::endl;
                set_depth_params = false;
                std::string stereo_sgbm_params_file = "../../config/stereo_sgbm_params_file.xml";
                cv::FileStorage fs(stereo_sgbm_params_file, cv::FileStorage::WRITE);
                fs << "min_disparity" << m_depthmapData.min_disparity;
                fs << "num_disparities" << m_depthmapData.num_disparities;
                fs << "blocksize" << m_depthmapData.blocksize;
                fs << "disp12_max_diff" << m_depthmapData.disp12_max_diff;
                fs << "uniqueness_ratio" << m_depthmapData.uniqueness_ratio;
                fs << "prev_speckle_window_size" << m_depthmapData.prev_speckle_window_size;
                fs << "speckle_range" << m_depthmapData.speckle_range;

                /*, m_depthmapData.min_disparity, m_depthmapData.num_disparities, m_depthmapData.blocksize, m_depthmapData.disp12_max_diff,
    m_depthmapData.uniqueness_ratio, m_depthmapData.prev_speckle_window_size, m_depthmapData.speckle_range*/
                cv::destroyAllWindows();
                break;
                //case ((int)('n')):
                //    cv::destroyAllWindows();
                //    // choose not to include params
                //    break;
            }
        }

        /* else
         {
             depthmap_data.stereo->compute(depthmap_data.left_nice, depthmap_data.right_nice, disp);
             cv::imshow(depthmap_data.window_name, disp);
             int key = cv::waitKey(100);

             switch (key)
             {

                 case ((int)('n')):
                     std::cout << "\nQutting application." << std::endl;
                     break;
                     return 0;
             }
         }*/


    }
    // Creating an object of StereoSGBM algorithm
   /* cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(min_disparity, num_disparities, blocksize, disp12_max_diff,
        uniqueness_ratio, prev_speckle_window_size, speckle_range);*/

        // Calculating disparith using the StereoSGBM algorithm
}

void StereoCalibration::onTrackbar(int, void* userdata)
{   
    DepthmapParams* depthmap_data = reinterpret_cast<DepthmapParams*>(userdata);
    
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Update Stereo Params: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    depthmap_data->stereo = cv::StereoSGBM::create(depthmap_data->min_disparity, 
                                                depthmap_data->num_disparities, depthmap_data->blocksize, 
                                                depthmap_data->disp12_max_diff, depthmap_data->uniqueness_ratio, 
                                                depthmap_data->prev_speckle_window_size, depthmap_data->speckle_range);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    std::cout << "Calculating new parameters:" << std::endl;


    if (depthmap_data->blocksize == 0) depthmap_data->blocksize = 1;
    if (depthmap_data->uniqueness_ratio == 0) depthmap_data->uniqueness_ratio = 1;
    if (depthmap_data->prev_speckle_window_size == 0) depthmap_data->prev_speckle_window_size = 1;
    if (depthmap_data->speckle_range == 0) depthmap_data->speckle_range = 1;

    try {

        cv::Mat disp, disp_color;
        depthmap_data->stereo->compute(depthmap_data->left_nice, depthmap_data->right_nice, disp); // left_nice, right_nice ( YOU CALCULATE A DEPTH MAP FROM A PAIR OF RECTIFIED IMAGES AFTER STEREO CALIBRATION!! )
        // https://albertarmea->com/post/opencv-stereo-camera/ (SEE TOWARDS END OF PAGE)

        // Normalizing the disparity map for better visualisation
        cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        depthmap_data->prev_min_disparity = depthmap_data->min_disparity;
        depthmap_data->prev_num_disparities = depthmap_data->num_disparities;
        depthmap_data->prev_blocksize = depthmap_data->blocksize;
        depthmap_data->prev_disp12_max_diff = depthmap_data->disp12_max_diff;
        depthmap_data->prev_uniqueness_ratio = depthmap_data->uniqueness_ratio;
        depthmap_data->prev_speckle_window_size = depthmap_data->prev_speckle_window_size;
        depthmap_data->prev_speckle_range = depthmap_data->speckle_range;

        // Displaying the disparity map
        cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);

        cv::imshow(depthmap_data->window_name, disp);
        cv::imshow(depthmap_data->window_name, disp_color);

        //cv::waitKey(1000);
        //cv::destroyAllWindows();
    }

    catch (cv::Exception) {
        fprintf(stderr, "Last value set threw an exception");
        depthmap_data->min_disparity = depthmap_data->prev_min_disparity;
        depthmap_data->num_disparities = depthmap_data->prev_num_disparities;
        depthmap_data->blocksize = depthmap_data->prev_blocksize;
        depthmap_data->disp12_max_diff = depthmap_data->prev_disp12_max_diff;
        depthmap_data->uniqueness_ratio = depthmap_data->prev_uniqueness_ratio;
        depthmap_data->prev_speckle_window_size = depthmap_data->prev_speckle_window_size;
        depthmap_data->speckle_range = depthmap_data->prev_speckle_range;
    }
}

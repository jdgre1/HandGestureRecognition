#include <camera_calibration.h>

StereoCalibration* StereoCalibration::instance = nullptr;

StereoCalibration::StereoCalibration(ros::NodeHandle& nh):
m_nh(nh)
{
    m_framesSaveDir = ros::package::getPath("camera_calibration") + "/data/";
    m_stereoParams.pre_stereo_calib_file =
        ros::package::getPath("camera_calibration") + "/config/pre_stereo_calib_file.xml";
};


StereoCalibration *StereoCalibration::GetInstance(ros::NodeHandle& nh)
{   
     if(instance==nullptr){
        instance = new StereoCalibration(nh);
    }
    return instance;
}

bool StereoCalibration::runParamServerConfig(void)
{
    bool paramSet = false;
    
    m_nh.getParam("/camera_calibration/check_frame_rate", paramSet);
    if(paramSet) 
    {   
        ROS_INFO("Checking frame rate");
        checkFrameRate(0, cv::CAP_ANY);
        checkFrameRate(1, cv::CAP_ANY);
    }

    m_nh.getParam("/camera_calibration/record_save", paramSet);
    if(paramSet) 
    {   
        ROS_INFO("Capture images for calibration");
        captureStereoImgsForCalibration();
    }

    m_nh.getParam("/camera_calibration/calibrate_single_cameras", paramSet);
    if(paramSet) 
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        bool saveParams = true;
        bool drawCorners = true;
        calibrateSingleCameras(saveParams, drawCorners);
    }

    m_nh.getParam("/camera_calibration/calibrate_stereo", paramSet);
    if(paramSet) 
    {   
        ROS_INFO("Calibrating cameras for stereovision");
        calibrateStereoCamera();
    }

    m_nh.getParam("/camera_calibration/test_calibration", paramSet);
    if(paramSet) 
    {   
        ROS_INFO("Testing calibration via live-image feed.\nRectifying image and undistorting.");
        testCalibration();
    }

    m_nh.getParam("/camera_calibration/run_depth_map", paramSet);
    if(paramSet) 
    {   
        ROS_INFO("Creating live depth-map.");
        createLiveDepthMap();
    }

    return 0;
}

void StereoCalibration::run()
{
    runParamServerConfig();
}

bool StereoCalibration::initialiseCameras()
{
    // Open both cameras:
    m_cap0.open(m_deviceID0, cv::CAP_ANY);
    m_cap1.open(m_deviceID1, cv::CAP_ANY);

    // check if we succeeded
    if (!m_cap0.isOpened() || !m_cap1.isOpened()) {
        std::cerr << "ERROR! Unable to open one or more cameras\n";
        return false;
    }

    // Begin capturing frames
    std::cout << "Capturing " << m_numCalibrationImgs << " frames for camera " + std::to_string(m_deviceID)
              << std::endl;

    return true;
}

int StereoCalibration::checkFrameRate(int deviceID, int apiID)
{
    // open selected camera using selected API
    cv::VideoCapture v_cap0;
    cv::VideoCapture v_cap1;
    v_cap0.open(deviceID, apiID);
    v_cap1.open(2, apiID);

    // check if we succeeded
    if (!v_cap0.isOpened() && !v_cap1.isOpened()) {
        std::cerr << "ERROR! Unable to open cameras \n";
        v_cap0.release();
        v_cap1.release();
        return -1;
    }

    double fps = v_cap0.get(cv::CAP_PROP_FPS);
    std::cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << std::endl;

    // Start and end times
    time_t start, end;

    // Variable for storing video frames
    cv::Mat frame;
    std::cout << "Capturing " << m_numImgsToTest << " frames" << std::endl;

    // Start time
    time(&start);

    // Grab a few frames
    for (int i = 0; i < m_numImgsToTest; i++) {
        v_cap0 >> frame;
    }

    // End Time
    time(&end);

    // Time elapsed
    double seconds = difftime(end, start);
    std::cout << "Time taken : " << seconds << " seconds" << std::endl;

    // Calculate frames per second
    fps = m_numImgsToTest / seconds;
    std::cout << "Estimated frames per second : " << fps << std::endl;

    // Release video
    v_cap0.release();
    return 0;
}

void StereoCalibration::captureStereoImgsForCalibration()
{
    // temp variables
    cv::Mat frame0, frame1;
    bool frameError;
    int frameCount;

    // loop time between consecutive frames
    time_t curr_time, prev;

    // start cameras
    if (initialiseCameras()) {
        // Start time
        time(&curr_time);
        time(&prev);

        // Incremental counter of successful frames
        frameCount = 0;
        // Set 1 frame every 2 seconds as the frame rate - (enough time to change position of chessboard)

        while (frameCount < m_numCalibrationImgs) {
            m_cap0 >> frame0;
            m_cap1 >> frame1;
            // check if we succeeded
            if (frame0.empty() || frame1.empty()) {
                ROS_ERROR("ERROR! blank frame of cam0 or cam1 grabbed!");
                continue;
            }

            time(&curr_time);
            if ((curr_time - prev >= 1. / m_frameRate) && checkCheckerboardCornersExist(frame0, frame1)) {
                frameCount++;
                prev = curr_time;
                std::string frame_num_str = std::to_string(frameCount);
                cv::imwrite(m_framesSaveDir + "0/frame0_" + std::to_string(frameCount) + ".png", frame0);
                cv::imwrite(m_framesSaveDir + "1/frame1_" + std::to_string(frameCount) + ".png", frame1);
                ROS_DEBUG_STREAM("frameCount: " << frameCount << std::endl);
            }
        }
    }
}

void StereoCalibration::calibrateSingleCameras(bool saveParams, bool drawCorners)
{

    processCheckerboardCorners(drawCorners);

    //  Single left camera calibration:
    // ~~~~~~~~~~~~~~~~~~~~~~
    fprintf(stderr, "Processing left camera calibration...\n");
    cv::Mat rvecs, tvecs;
    double rms = cv::calibrateCamera(m_stereoParams.objpoints_left, m_stereoParams.imgpoints_left, IMAGE_SIZE,
                                     m_stereoParams.mtxL, m_stereoParams.distL, rvecs, tvecs);
    std::cout << "Left camera calibration rms of " << rms << std::endl;

    //  Single right camera calibration:
    // ~~~~~~~~~~~~~~~~~~~~~~
    fprintf(stderr, "Processing right camera calibration...\n");
    rms = cv::calibrateCamera(m_stereoParams.objpoints_right, m_stereoParams.imgpoints_right, IMAGE_SIZE,
                              m_stereoParams.mtxR, m_stereoParams.distR, rvecs, tvecs);
    std::cout << "Right camera calibration rms of " << rms << std::endl;

    if (saveParams){
        saveCameraCalibParams();
    }
}

void StereoCalibration::processCheckerboardCorners(bool drawCorners)
{
    // Image Data
    int totalPhotos = 50;
    cv::Mat imgL, imgR, grayL, grayR;

    // Checkerboard settings
    cv::Vec3f pt(0, 0, 0);
    std::vector<cv::Vec3f> objp;
    cv::Size CHECKERBOARD(7, 10);
    for (int i = 0; i < CHECKERBOARD.height; i++){
        for (int j = 0; j < CHECKERBOARD.width; j++){
            objp.push_back(cv::Vec3f(j, i, 0));
        }
    }

    if (drawCorners){
        ROS_INFO("Press 'Q' to quit the script.\n");
    }
    
    for (uint16_t photoCounter = 1; photoCounter <= m_numCalibrationImgs; photoCounter++) {
        std::cout << "imgL: " << m_framesSaveDir + "0/frame0_" + std::to_string(photoCounter) + ".png" << std::endl;
        std::cout << "imgR: " << m_framesSaveDir + "1/frame1_" + std::to_string(photoCounter) + ".png" << std::endl;
        ROS_DEBUG("There are no images in pair No %d\n", photoCounter);
        imgL = cv::imread(m_framesSaveDir + "0/frame0_" + std::to_string(photoCounter) + ".png");
        imgR = cv::imread(m_framesSaveDir + "1/frame1_" + std::to_string(photoCounter) + ".png");

        if (imgR.empty() || imgL.empty()) {
            ROS_WARN("There are no images in pair No %d\n", photoCounter);
            continue;
        }

        // If stereopair is complete - go to processing
        cv::cvtColor(imgL, grayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR, grayR, cv::COLOR_BGR2GRAY);

        // Find the chessboard corners
        std::vector<cv::Vec2f> cornersL, cornersR;
        bool retL = cv::findChessboardCorners(grayL, CHECKERBOARD, cornersL,
                                              cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                                  cv::CALIB_CB_NORMALIZE_IMAGE);
        bool retR = cv::findChessboardCorners(grayR, CHECKERBOARD, cornersR,
                                              cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK +
                                                  cv::CALIB_CB_NORMALIZE_IMAGE);

        // Draw images with corners found
        if (drawCorners) {
            cv::drawChessboardCorners(imgL, CHECKERBOARD, cornersL, retL);
            cv::imshow("Corners LEFT", imgL);
            cv::drawChessboardCorners(imgR, CHECKERBOARD, cornersR, retR);
            cv::imshow("Corners RIGHT", imgR);
            char key = cv::waitKey();
            if (key == 'q' || key == 'Q'){
                exit(1);
            }
        }

        // Refine corners and add to array for processing
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);
        if (retL && retR) {
            m_stereoParams.objpoints_left.push_back(objp);
            cv::cornerSubPix(grayL, cornersL, cv::Size(3, 3), cv::Size(-1, -1), criteria);
            m_stereoParams.imgpoints_left.push_back(cornersL);
            m_stereoParams.objpoints_right.push_back(objp);
            cv::cornerSubPix(grayR, cornersR, cv::Size(3, 3), cv::Size(-1, -1), criteria);
            m_stereoParams.imgpoints_right.push_back(cornersR);
        } else {
            ROS_WARN("Pair No %d ignored, as no chessboard found\n", photoCounter);
            continue;
        }
    }
}

bool StereoCalibration::checkCheckerboardCornersExist(cv::Mat& frame0, cv::Mat& frame1)
{
    // Defining the dimensions of checkerboard
    int CHECKERBOARD[2]{7, 10};

    if (m_stereoParams.objp.size() == 0)
        for (int i{0}; i < CHECKERBOARD[1]; i++)
            for (int j{0}; j < CHECKERBOARD[0]; j++)
                m_stereoParams.objp.push_back(cv::Point3f(j, i, 0));

    cv::Mat gray0, gray1;
    std::vector<cv::Point2f> corner_pts0, corner_pts1;
    cv::cvtColor(frame0, gray0, cv::COLOR_BGR2GRAY);
    bool success = cv::findChessboardCorners(gray0, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts0,
                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                                 cv::CALIB_CB_NORMALIZE_IMAGE);
    std::cout << "Detected corners frame0? " << success << std::endl;
    cv::imshow("frame0", frame0);
    cv::imshow("frame1", frame1);
    cv::waitKey(1000);

    // If first frame successful, try other camera too:
    if (success) {
        cv::cvtColor(frame1, gray1, cv::COLOR_BGR2GRAY);
        success = cv::findChessboardCorners(gray1, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts1,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                                cv::CALIB_CB_NORMALIZE_IMAGE);
        std::cout << "Detected corners frame1? " << success << std::endl;
    }

    // If both successful, save points, draw checkboard corners and display
    if (success) {
        cv::Mat frame0Copy = frame0.clone();
        cv::Mat frame1Copy = frame1.clone();
        cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);

        cv::cornerSubPix(gray0, corner_pts0, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        cv::cornerSubPix(gray1, corner_pts1, cv::Size(11, 11), cv::Size(-1, -1), criteria);

        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame0Copy, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts0, success);
        cv::drawChessboardCorners(frame1Copy, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts1, success);

        // Display error between images on frames: Helps determine if images should be included for calibration:

        cv::imshow("frame0", frame0Copy);
        cv::imshow("frame1", frame1Copy);

        // Allow manual-user check:
        int key = cv::waitKey();
        switch (key) {
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

void StereoCalibration::calibrateStereoCamera()
{
    loadCameraCalibParams();

    m_stereoParams.new_mtxL =
        cv::getOptimalNewCameraMatrix(m_stereoParams.mtxL, m_stereoParams.distL, IMAGE_SIZE, 1, IMAGE_SIZE, 0);
    m_stereoParams.new_mtxR =
        cv::getOptimalNewCameraMatrix(m_stereoParams.mtxR, m_stereoParams.distR, IMAGE_SIZE, 1, IMAGE_SIZE, 0);

    cv::Mat Emat, Fmat;
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    double stereoRms = cv::stereoCalibrate(
        m_stereoParams.objpoints_right, m_stereoParams.imgpoints_left, m_stereoParams.imgpoints_right,
        m_stereoParams.new_mtxL, m_stereoParams.distL, m_stereoParams.new_mtxR, m_stereoParams.distR, IMAGE_SIZE,
        m_stereoParams.Rot, m_stereoParams.Trns, Emat, Fmat, flag,
        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));
    std::cout << "stereoRms: " << stereoRms << std::endl;
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
}

void StereoCalibration::rectifyAndUndistort()
{
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Rectification:
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cv::Mat rectifyLeft, rectifyRight, projMatLeft, projMatRight, Q;
    cv::stereoRectify(m_stereoParams.new_mtxL, m_stereoParams.distL, m_stereoParams.new_mtxR, m_stereoParams.distR,
                      IMAGE_SIZE, m_stereoParams.Rot, m_stereoParams.Trns, rectifyLeft, rectifyRight, projMatLeft, projMatRight, Q,
                      1);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Stereo Undistort:
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cv::initUndistortRectifyMap(m_stereoParams.new_mtxL, m_stereoParams.distL, rectifyLeft, projMatLeft, IMAGE_SIZE, CV_16SC2,
                                m_stereoParams.left_stereo_map1, m_stereoParams.left_stereo_map2);
    cv::initUndistortRectifyMap(m_stereoParams.new_mtxR, m_stereoParams.distR, rectifyRight, projMatRight, IMAGE_SIZE, CV_16SC2,
                                m_stereoParams.right_stereo_map1, m_stereoParams.right_stereo_map2);
}

void StereoCalibration::testCalibration()
{
    //  Load Test Images:
    cv::Mat imgLeft = cv::imread(m_framesSaveDir + "0/frame0_1.png");
    cv::Mat imgRight = cv::imread(m_framesSaveDir + "1/frame1_1.png");
    if(imgLeft.rows == 0 || imgRight.rows == 0)
    {
        ROS_ERROR("Invalid file directory ");
        return;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~

    //  Stereo Calibration:
    calibrateStereoCamera();
    // ~~~~~~~~~~~~~~~~~~~~~~

    //  Stereo Rectification / Undistortion:
    rectifyAndUndistort();
    // ~~~~~~~~~~~~~~~~~~~~~~

    //  Stereo Remap:
    cv::Mat leftAfterRectify, rightAfterRectify;
    cv::remap(imgLeft, leftAfterRectify, m_stereoParams.left_stereo_map1, m_stereoParams.left_stereo_map2, cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT, 0);
    cv::remap(imgRight, rightAfterRectify, m_stereoParams.right_stereo_map1, m_stereoParams.right_stereo_map2,
              cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    cv::imshow("Left image before rectification", imgLeft);
    cv::imshow("Right image before rectification", imgRight);

    cv::imshow("Left image after rectification", leftAfterRectify);
    cv::imshow("Right image after rectification", rightAfterRectify);

    cv::waitKey();
    ROS_INFO("Test complete!\n");
}

void StereoCalibration::createTrackbars()
{
    cv::namedWindow(m_depthmapData.window_name, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar(m_depthmapData.min_disparity_title, m_depthmapData.window_name, &m_depthmapData.min_disparity,
                       m_depthmapData.mindisparity_limit, onTrackbar);
    cv::createTrackbar(m_depthmapData.num_disparities_title, m_depthmapData.window_name,
                       &m_depthmapData.num_disparities, m_depthmapData.numdisparities_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.block_size_title, m_depthmapData.window_name, &m_depthmapData.blocksize,
                       m_depthmapData.blocksize_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.disp12_max_diff_title, m_depthmapData.window_name,
                       &m_depthmapData.disp12_max_diff, m_depthmapData.disp12maxdiff_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.uniqueness_ratio_title, m_depthmapData.window_name,
                       &m_depthmapData.uniqueness_ratio, m_depthmapData.uniquenessratio_max, onTrackbar);
    cv::createTrackbar(m_depthmapData.speckle_window_size_title, m_depthmapData.window_name,
                       &m_depthmapData.prev_speckle_window_size, m_depthmapData.prev_speckle_window_size, onTrackbar);
    cv::createTrackbar(m_depthmapData.speckle_range_title, m_depthmapData.window_name, &m_depthmapData.speckle_range,
                       m_depthmapData.specklerange_max, onTrackbar);
}

void StereoCalibration::createLiveDepthMap()
{
    calibrateStereoCamera();
    rectifyAndUndistort();

    // Stereo SGBM Initialisation:
    // ~~~~~~~~~~~~~~~~~~~~~~
    m_depthmapData.stereo =
        cv::StereoSGBM::create(m_depthmapData.min_disparity, m_depthmapData.num_disparities, m_depthmapData.blocksize,
                               m_depthmapData.disp12_max_diff, m_depthmapData.uniqueness_ratio,
                               m_depthmapData.prev_speckle_window_size, m_depthmapData.speckle_range);

    // Open Cameras and Read Images:
    // ~~~~~~~~~~~~~~~~~~~~~~
    bool frameError;
    bool setDepthParams = true;
    cv::Mat disp;

    if (!initialiseCameras())
        return;

    while (1) {
        m_cap0 >> m_depthmapData.orig_frame0;
        m_cap1 >> m_depthmapData.orig_frame1;
        frameError = false;
        // check if we succeeded
        if (m_depthmapData.orig_frame0.empty() || m_depthmapData.orig_frame1.empty()) {
            std::cerr << "Blank frame of cam0 or cam1 grabbed\n";
            frameError = true;
        }

        //  Stereo Remap:
        // ~~~~~~~~~~~~~~~~~~~~~~
        cv::remap(m_depthmapData.orig_frame0, m_depthmapData.left_after_rectify, m_depthmapData.left_stereo_map1,
                  m_depthmapData.left_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
        cv::remap(m_depthmapData.orig_frame1, m_depthmapData.right_after_rectify, m_depthmapData.right_stereo_map1,
                  m_depthmapData.right_stereo_map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

        // ~~~~~~~~~~~~~~~~~~~~~~

        cv::imshow("Left image before rectification", m_depthmapData.orig_frame0);
        cv::imshow("Right image before rectification", m_depthmapData.orig_frame1);

        cv::imshow("Left image after rectification", m_depthmapData.left_after_rectify);
        cv::imshow("Right image after rectification", m_depthmapData.right_after_rectify);

        if (setDepthParams) {
            createTrackbars();
            int key = cv::waitKey();

            switch (key) {
            case ((int)('y')):
                cv::waitKey(1000);
                cv::destroyAllWindows();
                std::cout << "\nParameters have been set." << std::endl;
                setDepthParams = false;
                std::string stereoSgbmParamsFile = "../../config/stereoSgbmParamsFile.xml";
                cv::FileStorage fs(stereoSgbmParamsFile, cv::FileStorage::WRITE);
                fs << "min_disparity" << m_depthmapData.min_disparity;
                fs << "num_disparities" << m_depthmapData.num_disparities;
                fs << "blocksize" << m_depthmapData.blocksize;
                fs << "disp12_max_diff" << m_depthmapData.disp12_max_diff;
                fs << "uniqueness_ratio" << m_depthmapData.uniqueness_ratio;
                fs << "prev_speckle_window_size" << m_depthmapData.prev_speckle_window_size;
                fs << "speckle_range" << m_depthmapData.speckle_range;

                /*, m_depthmapData.min_disparity, m_depthmapData.num_disparities, m_depthmapData.blocksize,
    m_depthmapData.disp12_max_diff, m_depthmapData.uniqueness_ratio, m_depthmapData.prev_speckle_window_size,
    m_depthmapData.speckle_range*/
                cv::destroyAllWindows();
                break;
                // case ((int)('n')):
                //     cv::destroyAllWindows();
                //     // choose not to include params
                //     break;
            }
        }

        /* else
         {
             depthmapData.stereo->compute(depthmapData.left_after_rectify, depthmapData.right_after_rectify, disp);
             cv::imshow(depthmapData.window_name, disp);
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
    /* cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(min_disparity, num_disparities, blocksize,
       disp12_max_diff, uniqueness_ratio, prev_speckle_window_size, speckle_range);*/

    // Calculating disparith using the StereoSGBM algorithm
}

void StereoCalibration::onTrackbar(int, void* userdata)
{
    DepthmapParams* depthmapData = reinterpret_cast<DepthmapParams*>(userdata);

    //  Update Stereo Params:
    // ~~~~~~~~~~~~~~~~~~~~~~
    depthmapData->stereo =
        cv::StereoSGBM::create(depthmapData->min_disparity, depthmapData->num_disparities, depthmapData->blocksize,
                               depthmapData->disp12_max_diff, depthmapData->uniqueness_ratio,
                               depthmapData->prev_speckle_window_size, depthmapData->speckle_range);

    // ~~~~~~~~~~~~~~~~~~~~~~

    ROS_DEBUG("Reloading disparity map with new input parameters:");

    if (depthmapData->blocksize == 0)
        depthmapData->blocksize = 1;
    if (depthmapData->uniqueness_ratio == 0)
        depthmapData->uniqueness_ratio = 1;
    if (depthmapData->prev_speckle_window_size == 0)
        depthmapData->prev_speckle_window_size = 1;
    if (depthmapData->speckle_range == 0)
        depthmapData->speckle_range = 1;

    try {

        cv::Mat disp, dispColor;
        depthmapData->stereo->compute(depthmapData->left_after_rectify, depthmapData->right_after_rectify,
                                       disp); // leftAfterRectify, right_after_rectify ( YOU CALCULATE A DEPTH MAP FROM A PAIR OF
                                              // RECTIFIED IMAGES AFTER STEREO CALIBRATION!! )
        // https://albertarmea->com/post/opencv-stereo-camera/ (SEE TOWARDS END OF PAGE)

        // Normalizing the disparity map for better visualisation
        cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        depthmapData->prev_min_disparity = depthmapData->min_disparity;
        depthmapData->prev_num_disparities = depthmapData->num_disparities;
        depthmapData->prev_blocksize = depthmapData->blocksize;
        depthmapData->prev_disp12_max_diff = depthmapData->disp12_max_diff;
        depthmapData->prev_uniqueness_ratio = depthmapData->uniqueness_ratio;
        depthmapData->prev_speckle_window_size = depthmapData->prev_speckle_window_size;
        depthmapData->prev_speckle_range = depthmapData->speckle_range;

        // Displaying the disparity map
        cv::applyColorMap(disp, dispColor, cv::COLORMAP_JET);

        cv::imshow(depthmapData->window_name, disp);
        cv::imshow(depthmapData->window_name, dispColor);

        // cv::waitKey(1000);
        // cv::destroyAllWindows();
    }

    catch (cv::Exception) {
        ROS_WARN("Depth map trackbar-data was invalid!");
        depthmapData->min_disparity = depthmapData->prev_min_disparity;
        depthmapData->num_disparities = depthmapData->prev_num_disparities;
        depthmapData->blocksize = depthmapData->prev_blocksize;
        depthmapData->disp12_max_diff = depthmapData->prev_disp12_max_diff;
        depthmapData->uniqueness_ratio = depthmapData->prev_uniqueness_ratio;
        depthmapData->prev_speckle_window_size = depthmapData->prev_speckle_window_size;
        depthmapData->speckle_range = depthmapData->prev_speckle_range;
    }
}



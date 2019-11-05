#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW_NAME	"Autonomous Grasping"
// function headers
void UIButtons(void);
void OffsetsWindow(void);
void updateDialog();
void CheckMouse();
void drawCubeWireFrame(
        cv::InputOutputArray image, cv::InputArray cameraMatrix,
        cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
        float l
);


//global variables
static cv::Mat frame = cv::Mat(600, 1024, CV_8UC3);
static int state = 0;


int main(int argc, const char *argv[])
{
    int wait_time = 10;
    float actual_marker_l = 0.101; // this should be in meters

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::VideoCapture in_video;

    in_video.open(0);

    cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    cv::FileStorage fs("../calibration_params.yml", cv::FileStorage::READ);
//    cv::FileStorage fs(argv[2], cv::FileStorage::READ);

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    //camera calibration data
    std::cout << "camera_matrix\n"
              << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n"
              << dist_coeffs << std::endl;


    // Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
    cvui::init(WINDOW_NAME);
    

    while (in_video.grab()){ //Loop while video exists
//Start Image Processing
        in_video.retrieve(image);

        image.copyTo(image_copy);

        frame = cv::Scalar(40, 40, 40); //set UI color

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                    corners, actual_marker_l, camera_matrix, dist_coeffs,
                    rvecs, tvecs
            );

            // draw axis for each marker
            for (int i = 0; i < ids.size(); i++)
            {
                drawCubeWireFrame(
                        image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i],
                        actual_marker_l
                );
            }
        }
//End Image Processing

        cvui::image(frame, 375, 10, image_copy);

        updateDialog(); //update the dialog box

        UIButtons(); //display and check buttons

        CheckMouse(); //monitor the mouse input

        OffsetsWindow(); //display offsets window


        // Show everything on the screen
        cvui::update();
        cv::imshow(WINDOW_NAME, frame);

        // Check if ESC key was pressed
        if (cv::waitKey(20) == 27) {
            break;
        }
    }
    in_video.release();

    return 0;
}




void drawCubeWireFrame(
        cv::InputOutputArray image, cv::InputArray cameraMatrix,
        cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
        float l
)
{

    CV_Assert(
            image.getMat().total() != 0 &&
            (image.getMat().channels() == 1 || image.getMat().channels() == 3)
    );
    CV_Assert(l > 0);
    float half_l = l / 2.0;
    l /=2;
    // project cube points
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(half_l, half_l, l));
    axisPoints.push_back(cv::Point3f(half_l, -half_l, l));
    axisPoints.push_back(cv::Point3f(-half_l, -half_l, l));
    axisPoints.push_back(cv::Point3f(-half_l, half_l, l));
    axisPoints.push_back(cv::Point3f(half_l, half_l, 0));
    axisPoints.push_back(cv::Point3f(half_l, -half_l, 0));
    axisPoints.push_back(cv::Point3f(-half_l, -half_l, 0));
    axisPoints.push_back(cv::Point3f(-half_l, half_l, 0));

    std::vector<cv::Point2f> imagePoints;
    projectPoints(
            axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints
    );

    // draw cube edges lines
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[4], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[2], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[3], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[5], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), 3);
}



void UIButtons(){
    if (cvui::button(frame, 30, 80,120,40 ,  "&Pick")) {
        state = 0;
    }
    if (cvui::button(frame, 180, 80,120,40 , "&Place")) {
        state = 1;
    }

    if (cvui::button(frame, 500, 500,120,40 ,  "&Act")) {

    }
    if (cvui::button(frame, 650, 500,120,40 , "&Cancel")) {

    }

    if (cvui::button(frame, 500, 550,120,40 ,  "&Reset")) {

    }
    if (cvui::button(frame, 650, 550,120,40 , "&Home")) {

    }
    if (cvui::button(frame, 30, 550, 150, 40,"&Auxiliary Camera")){
        //launch secondary window and display camera images there
    }
}



void OffsetsWindow(){

    cvui::rect(frame, 10, 200, 300, 300, 0xffffff);
    cvui::text(frame, 45, 250, "Where It Be ", 0.4, 0xffffff);

    //x pos
    cvui::rect(frame, 50, 270, 60, 20, 0xffffff);
    cvui::printf(frame, 55, 275, "%d", 100);
    //y pos
    cvui::rect(frame, 50, 310, 60, 20, 0xffffff);
    cvui::printf(frame, 55, 315, "%d", 100);
    //z pos
    cvui::rect(frame, 50, 350, 60, 20, 0xffffff);
    cvui::printf(frame, 55, 355, "%d", 100);

    cvui::text(frame, 185, 250, "Twisty Boiz", 0.4, 0xffffff);
    cvui::rect(frame, 190, 270, 60, 20, 0xffffff);
    cvui::printf(frame, 195, 275, "%d", 100);
    cvui::rect(frame, 190, 310, 60, 20, 0xffffff);
    cvui::printf(frame, 195, 315, "%d", 100);
    cvui::rect(frame, 190, 350, 60, 20, 0xffffff);
    cvui::printf(frame, 195, 355, "%d", 100);
}

void updateDialog(){
    if(state == 0) {
        cvui::text(frame, 20, 30, "Please select marker to pick up ", 0.5, 0xffffff);
    }
    else if(state == 1){
        cvui::text(frame, 20, 30, "Please select placement marker ", 0.5, 0xffffff);
    }
}

void CheckMouse(){
    if (cvui::mouse(cvui::CLICK)) {
        //cvui::text(image_copy, 10, 70, "Mouse was clicked!");
        if((cvui::mouse().x>=375)&&((cvui::mouse().y>=10)&&(cvui::mouse().y<=490))) {

            //if pick up marker exists in this area & is selected and state == 0
            if (state == 0) {
                printf("Picking Up at %d %d \n", cvui::mouse().x, cvui::mouse().y);
            }
            else{
                printf("Placing Up at %d %d \n", cvui::mouse().x, cvui::mouse().y);

            }

        }
    }
}
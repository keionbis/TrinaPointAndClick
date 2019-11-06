#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "AuxCameraDisplay.h"

#define WINDOW_NAME	"Autonomous Grasping"

// function declarations
void UIButtons(void);
void OffsetsWindow(void);
void updateDialog();
void CheckMouse();
void drawCubeWireFrame(
        cv::InputOutputArray image, cv::InputArray cameraMatrix,
        cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
        float l
);
void publishAllTheRos(void);

typedef struct Marker{
    int ID;
    int timesSeen;
    float location[4]; //xmin, xmax, ymin, ymax

}Marker;


//global variables
static cv::Mat frame = cv::Mat(600, 1024, CV_8UC3);
static int state = 0;
bool AuxCameraOpen = false;
Marker Markers[512];
std::vector<Marker> ConfirmedMarkers;
Marker tmpData;
Marker PickLocation = {0xFFFFFF, 0, {0,0,0,0}};;
Marker PlaceLocation = {0xFFFFFF, 0, {0,0,0,0}};;
Marker ClearSet = {0xFFFFFF, 0, {0,0,0,0}};

int main(int argc, const char *argv[])
{
    int wait_time = 10;
    float actual_marker_l = 0.101; // this should be in meters
    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::VideoCapture in_video;

    in_video.open(0);//Camera index should be a passed parameter

    cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    cv::FileStorage fs("../calibration_params.yml", cv::FileStorage::READ); //hard coded calibration file
//    cv::FileStorage fs(argv[2], cv::FileStorage::READ); //parameter passes calibration file

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

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

                if(ids[i]<512){
                    Markers[ids[i]] = {ids[i], 0, {corners[i][0].x, corners[i][3].x, corners[i][0].y, corners[i][3].y}};
                }
                drawCubeWireFrame(
                        image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i],
                        actual_marker_l
                );
            }
        }
//End Image Processing

        cvui::image(frame, 375, 10, image_copy); //Display image into the interface

        updateDialog(); //update the dialog box

        UIButtons(); //display and check buttons

        CheckMouse(); //monitor the mouse input

        OffsetsWindow(); //display offsets window

        publishAllTheRos(); //exactly what the function name says
        if(AuxCameraOpen){
            monitorAuxCam();
        }

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
        //stop publishers running
    }
    if (cvui::button(frame, 650, 500,120,40 , "&Cancel")) {
        PickLocation  = ClearSet;
        PlaceLocation = ClearSet;
        printf("%d, %d, %.2f, %.2f, %.2f, %.2f \n\r", PickLocation.ID, PickLocation.timesSeen, PickLocation.location[0],PickLocation.location[1], PickLocation.location[2], PickLocation.location[3]);
        printf("%d, %d, %.2f, %.2f, %.2f, %.2f \n\r", PlaceLocation.ID, PlaceLocation.timesSeen, PlaceLocation.location[0],PlaceLocation.location[1], PlaceLocation.location[2], PlaceLocation.location[3]);
        //stop publishers running
    }

    if (cvui::button(frame, 500, 550,120,40 ,  "&Reset")) {

    }
    if (cvui::button(frame, 650, 550,120,40 , "&Home")) {
        //tell robot to go to its neutral pose
    }
    if ((cvui::button(frame, 30, 550, 150, 40,"&Auxiliary Camera"))&&(!AuxCameraOpen)){
        //launch secondary window and display camera images there
        //StartCameraDisplay();
        //AuxCameraOpen = true;
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
            int mouseX = cvui::mouse().x-375;
            int mouseY = cvui::mouse().y-10;
            //if pick up marker exists in this area & is selected and state == 0
            if (state == 0) {
                printf("Picking Up at %d %d \n", mouseX, mouseY);


                PickLocation = {10, 0, {1,2,3,4}};
            }
            else{
                printf("Placing at %d %d \n",  mouseX, mouseY);
                PlaceLocation = {20, 0, {5,6,7,8}};

            }
            printf("%d, %d, %.2f, %.2f, %.2f, %.2f \n\r", PickLocation.ID, PickLocation.timesSeen, PickLocation.location[0],PickLocation.location[1], PickLocation.location[2], PickLocation.location[3]);
            printf("%d, %d, %.2f, %.2f, %.2f, %.2f \n\r", PlaceLocation.ID, PlaceLocation.timesSeen, PlaceLocation.location[0],PlaceLocation.location[1], PlaceLocation.location[2], PlaceLocation.location[3]);

        }
    }
}



void publishAllTheRos(void){

}

Marker checkifMarkerExists(int ID){
    std::vector<Marker>::iterator ite;

    //ite = std::find_if(IDsSeen.begin(), IDsSeen.end(),  [](Marker& f){ return f.ID == ID; } );
    return *ite;
}

void appendMarkerToVector(int ID){
    Marker marker = checkifMarkerExists(ID);
    if(marker.ID == 0 && marker.timesSeen == 0 && marker.location[0]  == marker.location[1] ){//if response was null

    }
}




#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "AuxCameraDisplay.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/builtin_bool.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#define WINDOW_NAME	"Autonomous Grasping"
#define CONFIRMATION 50
#define MAXCLICKERROR 50
typedef struct Marker{
    int ID;
    int timesSeen;
    cv::Point2f location[4]; //xmin, xmax, ymin, ymax
    cv::Point2f centroid; //xmin, xmax, ymin, ymax

}Marker;

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
void publishAllTheRos(const ros::TimerEvent&);
void checkifMarkerExists(Marker marker);
void MarkerIsReliable(Marker marker);
int LocateNearestMarker(cv::Point2f clickLocation);
void initialisePublishers();
void initialiseSubscribers();

//global variables
static cv::Mat frame = cv::Mat(600, 1024, CV_8UC3);
static int state = 0;
bool AuxCameraOpen = false;
bool ApplyOffsets = false;
bool LiveOffsets = false;
int XOffset = 0; //in mm
int YOffset = 0; //in mm
int ZOffset = 0; //in mm
int RollOffset = 0; //in degrees
int PitchOffset = 0; //in degrees
int YawOffset = 0; //in degrees
bool GripperOpen = false;
cv::String GripperState = "Open Gripper";
int ClosePercent = 85; //Gripper close percent
int CloseSpeedPercent = 20; //Gripper close speed percent
std::vector<Marker> Markers;
std::vector<Marker> ConfirmedIDs;
Marker tmpData;
Marker PickLocation ;
Marker PlaceLocation;

std_msgs::Bool currentRobotStatus; //is the robot doing things


ros::Publisher MarkerPose;
ros::Publisher GripperSpeed;
ros::Publisher GripperClosePercent;
ros::Publisher PickID;
ros::Publisher PlaceID;
ros::Publisher Command;
ros::Publisher GripperStatepub;
ros::Subscriber currentStatus;
void CurrentStatusCallback(const std_msgs::Bool::ConstPtr& msg){
    //currentRobotStatus = msg;
}


int main(int argc, char *argv[])
{
    int wait_time = 10;
    float actual_marker_l = 0.101; // this should be in meters
    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

     MarkerPose = n.advertise<geometry_msgs::PoseStamped>("MarkerPose", 1000);
     GripperSpeed = n.advertise<std_msgs::Int64>("GripperSpeed", 1000);
     GripperClosePercent = n.advertise<std_msgs::Int64>("GripperClosePercent", 1000);
     GripperStatepub = n.advertise<std_msgs::Int64>("GripperState", 1000);
     PickID = n.advertise<std_msgs::Int64>("PickID", 1000);
     PlaceID = n.advertise<std_msgs::Int64>("PlaceID", 1000);
     Command = n.advertise<std_msgs::String>("Command", 1000);

    currentStatus = n.subscribe("CurrentStatus", 1000, CurrentStatusCallback);

    ros::Timer Publisher = n.createTimer(ros::Duration(0.1), publishAllTheRos);

    cv::VideoCapture in_video;

    in_video.open(0);//Camera index should be a passed parameter

    cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    cv::FileStorage fs("../../../src/Trina-Point-And-Click/calibration_params.yml", cv::FileStorage::READ); //hard coded calibration file
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
               tmpData = {ids[i], 0, {corners[i][0],corners[i][1], corners[i][2], corners[i][3]}, {0,0}};
                checkifMarkerExists(tmpData);
                drawCubeWireFrame(
                        image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i],
                        actual_marker_l
                );

//                Transform.header.stamp = ros::Time::now();
//                Transform.header.frame_id = std::to_string(ids[i]);
//                Transform.child_frame_id = "detected";
//                Transform.transform.rotation.w = 0;
//                Transform.transform.rotation.x = rvecs[i][0];
//                Transform.transform.rotation.y = rvecs[i][1];
//                Transform.transform.rotation.z = rvecs[i][2];
//                Transform.transform.translation.x = tvecs[i][0];
//                Transform.transform.translation.y = tvecs[i][1];
//                Transform.transform.translation.z = tvecs[i][2];
//
//                MarkerTracker_Transform_pub.publish(Transform);

            }
        }
//End Image Processing

        cvui::image(frame, 375, 10, image_copy); //Display image into the interface

        updateDialog(); //update the dialog box

        UIButtons(); //display and check buttons

        CheckMouse(); //monitor the mouse input

        OffsetsWindow(); //display offsets window

        if(AuxCameraOpen){
            monitorAuxCam();
        }

        // Show everything on the screen
        cvui::update();
        cv::imshow(WINDOW_NAME, frame);

        // Check if ESC key was pressed
        if (cv::waitKey(20) == 27|| cv::getWindowProperty(WINDOW_NAME, cv::WND_PROP_ASPECT_RATIO) < 0) {
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
        //stop publishers running
    }

    if (cvui::button(frame, 500, 550,120,40 ,  "&Reset")) {

    }
    if (cvui::button(frame, 650, 550,120,40 , "&Home")) {
        //tell robot to go to its neutral pose
    }
    if ((cvui::button(frame, 850, 550, 150, 40,"&Auxiliary Camera"))&&(!AuxCameraOpen)){
        //launch secondary window and display camera images there
//        StartCameraDisplay();
//        AuxCameraOpen = true;
    }
}



void OffsetsWindow(){

    //offset box
    cvui::rect(frame, 10, 150, 350, 210, 0xffffff);
    cvui::text(frame, 125, 165, "Gripper Offsets", 0.5, 0xffffff);

    //position
    cvui::beginColumn(frame, 40, 200, 145, 160,10);

        cvui::checkbox("Apply offsets", &ApplyOffsets, 0xffffff);
        cvui::space(5);
        cvui::text("Position (mm):", 0.4, 0xffffff);

        //x
        cvui::beginRow(-1,-1, 5);
            cvui::text("x:", 0.5, 0xffffff);
            cvui::counter(&XOffset,10,"%d");
        cvui::endRow();

        //y
        cvui::beginRow(-1,-1,5);
            cvui::text("y:", 0.5, 0xffffff);
            cvui::counter(&YOffset,10,"%d");
        cvui::endRow();

        //z
        cvui::beginRow(-1,-1,5);
            cvui::text("z:", 0.5, 0xffffff);
            cvui::counter(&ZOffset,10,"%d");
        cvui::endRow();

    cvui::endColumn();

    //orientation
    cvui::beginColumn(frame, 200, 200, 145, 160,10);

        cvui::checkbox("Live adjustments", &LiveOffsets, 0xffffff);
        cvui::space(5);
        cvui::text("Rotation (deg):", 0.4, 0xffffff);

        //roll
        cvui::beginRow(-1,-1, 5);
            cvui::text("Rx:", 0.5, 0xffffff);
            cvui::counter(&RollOffset,45,"%d");
        cvui::endRow();

        //pitch
        cvui::beginRow(-1,-1,5);
            cvui::text("Ry:", 0.5, 0xffffff);
            cvui::counter(&PitchOffset,45,"%d");
        cvui::endRow();

        //yaw
        cvui::beginRow(-1,-1,5);
            cvui::text("Rz:", 0.5, 0xffffff);
            cvui::counter(&YawOffset,45,"%d");
        cvui::endRow();

    cvui::endColumn();

    //gripper box
    cvui::rect(frame, 10, 380, 350, 210, 0xffffff);
    cvui::text(frame, 125, 395, "Gripper Control", 0.5, 0xffffff);

    cvui::beginColumn(frame, 30, 430, 310, 160,10);

        //gripper toggle
        cvui::beginRow(-1,-1,10);
            cvui::space(40);
            cvui::space(35);
           if(cvui::button(120, 40, GripperState)){
                if (GripperOpen){
                    GripperState = "Close Gripper";
                }
                else{
                    GripperState = "Open Gripper";
                }
                GripperOpen =! GripperOpen;

            }

        cvui::endRow();

        // gripper close
        cvui::beginRow(1,-1,9);
            cvui::text("Gripper Close %", 0.4, 0xffffff);
            cvui::trackbar(200, &ClosePercent, 70, 100, 6, "%.0Lf", cvui::TRACKBAR_DISCRETE, 5);
        cvui::endRow();

        // gripper close
        cvui::beginRow(1,-1,5);
            cvui::text("Gripper speed %", 0.4, 0xffffff);
            cvui::trackbar(200, &CloseSpeedPercent, 0, 100, 5, "%.0Lf", cvui::TRACKBAR_DISCRETE, 10);
        cvui::endRow();

    cvui::endColumn();
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
                LocateNearestMarker({mouseX, mouseY});

            }
            else {
                printf("Placing at %d %d \n", mouseX, mouseY);

            }

        }
    }
}



void publishAllTheRos(const ros::TimerEvent&){

}

void checkifMarkerExists(Marker marker){
    for(std::vector<Marker>::iterator it = Markers.begin() ; it != Markers.end(); ++it){
        if (it->ID == marker.ID){
            it->timesSeen++;
            if(it->timesSeen>=CONFIRMATION){
                MarkerIsReliable(marker);
            }
            return;
        }
    }
    Markers.push_back(marker);
}

void MarkerIsReliable(Marker marker){
    for(std::vector<Marker>::iterator it = ConfirmedIDs.begin() ; it != ConfirmedIDs.end(); ++it){
        if (it->ID == marker.ID){
            it->location[0] = marker.location[0];
            it->location[1] = marker.location[1];
            it->location[2] = marker.location[2];
            it->location[3] = marker.location[3];
            it->centroid.x = (marker.location[0].x+marker.location[1].x+marker.location[2].x+marker.location[3].x)/4;
            it->centroid.y = (marker.location[0].y+marker.location[1].y+marker.location[2].y+marker.location[3].y)/4;
            return;
        }
    }
    ConfirmedIDs.push_back(marker);
}

int LocateNearestMarker(cv::Point2f clickLocation) {
    int nearestID;
    cv::Point2f NearestMarerLocation = {0,0};
    float disttoNearest = ((sqrt(pow((NearestMarerLocation.x-clickLocation.x),2))+(pow((NearestMarerLocation.y-clickLocation.y), 2))));
    for (std::vector<Marker>::iterator it = ConfirmedIDs.begin(); it != ConfirmedIDs.end(); it++) {
        float disttoCurrentMarer =(sqrt(pow((it->centroid.x-clickLocation.x), 2))+(pow((it->centroid.y-clickLocation.x), 2)));
        if(disttoCurrentMarer<disttoNearest && disttoCurrentMarer<MAXCLICKERROR){
            disttoNearest = disttoCurrentMarer;
            nearestID = it->ID;
        }
    }
    return nearestID;
}


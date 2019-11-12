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
#include "geometry_msgs/Pose.h"
#define WINDOW_NAME	"Autonomous Grasping"
#define CONFIRMATION 50
#define MAXCLICKERROR 10000

//Marker Data Struct definition
typedef struct Marker{
    int ID;
    int timesSeen;
    cv::Point2f location[4]; //xmin, xmax, ymin, ymax
    cv::Point2f centroid; //xmin, xmax, ymin, ymax
    bool isVisible;
}Marker;

// function declarations
void UIButtons(void);
void OffsetsWindow(void);
void updateDialog();
void CheckMouse();
void drawCubeWireFrame(
        cv::InputOutputArray image, cv::InputArray cameraMatrix,
        cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
        float l, int id
);
void publishAllTheRos();
void checkifMarkerExists(Marker marker);
void MarkerIsReliable(Marker marker);
int LocateNearestMarker(cv::Point2f clickLocation);
void initialisePublishers();
void initialiseSubscribers();
void checkReady();

//GLOBAL VARIABLES

//Marker vectors
std::vector<Marker> Markers;
std::vector<Marker> ConfirmedIDs;
//CV Variables
static cv::Mat frame = cv::Mat(600, 1024, CV_8UC3);
std::string Pick = "Click 'Pick'.";
std::string Place = "Click 'Place'.";
std::string Wrong = "No marker near selection. Try clicking a different spot!";
std::string Picking = "Click a marker to pick up.";
std::string Placing = "Click a marker to place.";
std::string Doing = "Robot in action now";
std::string Done = "I did it! Now click 'Pick'.";
std::string Ready = "Click 'Act' to make the robot do stuff.";
static std::string state = Pick;
std_msgs::String command;
geometry_msgs::Pose Offsets;
geometry_msgs::PoseStamped PoseStamped;
bool currentRobotStatus = false;

//Publisher state variables
bool AuxCameraOpen = false;
bool ApplyOffsets = false;
bool LiveOffsets = false;
bool GripperOpen = false;

//offset and gripper variables
int XOffset = 0, YOffset = 0, ZOffset = 0; //in mm
int RollOffset = 0, PitchOffset = 0, YawOffset = 0; //in degrees
cv::String GripperState = "Open Gripper";
int ClosePercent = 85, CloseSpeedPercent = 20; //Gripper data

Marker tmpData;
int PickID = 2512;
int PlaceID = 7320;

//ROS Publishers
ros::Publisher MarkerPosePublisher;
ros::Publisher GripperSpeedPublisher;
ros::Publisher GripperClosePercentPublisher;
ros::Publisher PickIDPublisher;
ros::Publisher PlaceIDPublisher;
ros::Publisher CommandPublisher;
ros::Publisher GripperStatePublisher;
ros::Publisher OffsetPublisher;
//ROS Subscribers
ros::Subscriber currentStatusSubscriber;

void CurrentStatusCallback(const std_msgs::Bool::ConstPtr& Status){
    //read in data being published about whether the task is complete or on-going
    currentRobotStatus = Status->data;
    //checks if action was completed
    if (state == Doing){
        if (currentRobotStatus){
            state = Done;
            PickID = 2512;
            PlaceID = 7320;
        }
    }
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
    command.data= "wait";
     MarkerPosePublisher = n.advertise<geometry_msgs::PoseStamped>("MarkerPose", 1000);
     GripperSpeedPublisher = n.advertise<std_msgs::Int64>("GripperSpeed", 1000);
     GripperClosePercentPublisher = n.advertise<std_msgs::Int64>("GripperClosePercent", 1000);
     GripperStatePublisher = n.advertise<std_msgs::String>("GripperState", 1000);
     PickIDPublisher = n.advertise<std_msgs::Int64>("PickID", 1000);
     PlaceIDPublisher = n.advertise<std_msgs::Int64>("PlaceID", 1000);
     CommandPublisher = n.advertise<std_msgs::String>("Command", 1000);
    OffsetPublisher = n.advertise<geometry_msgs::Pose>("Offsets", 1000);

    currentStatusSubscriber = n.subscribe("CurrentStatus", 1000, CurrentStatusCallback);

    //ros::Timer timer1 = n.createTimer(ros::Duration(1), publishAllTheRos);
    cv::VideoCapture in_video;

    in_video.open(0);//Camera index should be a passed parameter

    cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    cv::FileStorage fs("../../../src/Trina-Point-And-Click/calibration_params.yml", cv::FileStorage::READ); //hard coded calibration file
    //cv::FileStorage fs(argv[2], cv::FileStorage::READ); //parameter passes calibration file

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    // Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
    cvui::init(WINDOW_NAME);


    while (in_video.grab()&& ros::ok()){ //Loop while video exists
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
            for(int x = 0; x<ConfirmedIDs.size();x++){
                ConfirmedIDs[x].isVisible = false;
            }
            // draw axis for each marker
            for (int i = 0; i < ids.size(); i++)
            {
               tmpData = {ids[i], 0, {corners[i][0],corners[i][1], corners[i][2], corners[i][3]}, {0,0}, true};
                checkifMarkerExists(tmpData);
                drawCubeWireFrame(
                        image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i],
                        actual_marker_l, ids[i]
                );
                if(ApplyOffsets && !LiveOffsets) {
                    PoseStamped.header.stamp = ros::Time::now();
                    PoseStamped.header.frame_id = std::to_string(ids[i]);
                    PoseStamped.pose.orientation.w = 0;
                    PoseStamped.pose.orientation.x = rvecs[i][0]+(XOffset/1000);
                    PoseStamped.pose.orientation.y = rvecs[i][1]+(YOffset/1000);
                    PoseStamped.pose.orientation.z = rvecs[i][2]+(ZOffset/1000);
                    PoseStamped.pose.position.x = tvecs[i][0]+RollOffset;
                    PoseStamped.pose.position.y = tvecs[i][1]+PitchOffset;
                    PoseStamped.pose.position.z = tvecs[i][2]+YawOffset;
                }
                else{
                    PoseStamped.header.stamp = ros::Time::now();
                    PoseStamped.header.frame_id = std::to_string(ids[i]);
                    PoseStamped.pose.orientation.w = 0;
                    PoseStamped.pose.orientation.x = rvecs[i][0];
                    PoseStamped.pose.orientation.y = rvecs[i][1];
                    PoseStamped.pose.orientation.z = rvecs[i][2];
                    PoseStamped.pose.position.x = tvecs[i][0];
                    PoseStamped.pose.position.y = tvecs[i][1];
                    PoseStamped.pose.position.z = tvecs[i][2];
                }

                MarkerPosePublisher.publish(PoseStamped);
            }
        }
//End Image Processing
        //Display image into the interface
        cvui::image(frame, 375, 10, image_copy);
        //update the dialog box
        updateDialog();
        //display and check buttons
        UIButtons();
        //monitor the mouse input
        CheckMouse();
        //display offsets window
        OffsetsWindow();
        // publish all the ros topics
        publishAllTheRos();

//        if(AuxCameraOpen){
//            monitorAuxCam();
//        }

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
        float l, int id){
    for(std::vector<Marker>::iterator it = ConfirmedIDs.begin() ; it != ConfirmedIDs.end(); ++it){
        if(it->ID == id) {
            CV_Assert(
                    image.getMat().total() != 0 &&
                    (image.getMat().channels() == 1 || image.getMat().channels() == 3)
            );
            CV_Assert(l > 0);
            float half_l = l / 2.0;
            l /= 2;

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
            cv::Scalar blue (255, 0, 0);
            cv::Scalar yellow (0, 255, 255);
            cv::Scalar teal (255, 2250,0 );

            cv::Scalar color = blue;


            if(id == PickID ){
                color = yellow;
            }
            else if(id == PlaceID){
                color = teal;

            }
            // draw cube edges lines
            cv::line(image, imagePoints[0], imagePoints[1], color, 3);
            cv::line(image, imagePoints[0], imagePoints[3], color, 3);
            cv::line(image, imagePoints[0], imagePoints[4], color, 3);
            cv::line(image, imagePoints[1], imagePoints[2], color, 3);
            cv::line(image, imagePoints[1], imagePoints[5], color, 3);
            cv::line(image, imagePoints[2], imagePoints[3], color, 3);
            cv::line(image, imagePoints[2], imagePoints[6], color, 3);
            cv::line(image, imagePoints[3], imagePoints[7], color, 3);
            cv::line(image, imagePoints[4], imagePoints[5], color, 3);
            cv::line(image, imagePoints[4], imagePoints[7], color, 3);
            cv::line(image, imagePoints[5], imagePoints[6], color, 3);
            cv::line(image, imagePoints[6], imagePoints[7], color, 3);
            return;
        }
    }
}

void UIButtons(){
    if (cvui::button(frame, 30, 80,120,40 ,  "&Pick")) {
        state = Picking;
    }

    if (cvui::button(frame, 180, 80,120,40 , "&Place")) {
        state = Placing;
    }

    if (cvui::button(frame, 500, 500,120,40 ,  "&Act")) {
        //stop publishers running
        if (state == Ready){
            state = Doing;
            command.data = "act";
        }
    }
    if (cvui::button(frame, 650, 500,120,40 , "&Cancel")) {
        //stop publishers running
        state = Pick;
        command.data = "cancel";
    }

    if (cvui::button(frame, 500, 550,120,40 ,  "&Reset")) {
        state = Pick;
        PickID = 2512;
        PlaceID = 7320;
        currentRobotStatus = false;

        //Publisher state variables
        AuxCameraOpen = false;
        ApplyOffsets = false;
        LiveOffsets = false;
        GripperOpen = false;

        //offset and gripper variables
        XOffset = 0, YOffset = 0, ZOffset = 0; //in mm
        RollOffset = 0, PitchOffset = 0, YawOffset = 0; //in degrees
        GripperState = "Open Gripper";
        //Gripper data
        ClosePercent = 85;
        CloseSpeedPercent = 20;
    }
    if (cvui::button(frame, 650, 550,120,40 , "&Home")) {
        //tell robot to go to its neutral pose
        command.data = "home";
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

        if(cvui::checkbox("Live adjustments", &LiveOffsets, 0xffffff) && ApplyOffsets){
            Offsets.position.x = double(XOffset/1000);
            Offsets.position.y = double(YOffset/1000);
            Offsets.position.z = double(ZOffset/1000);
            Offsets.orientation.x = RollOffset;
            Offsets.orientation.y = PitchOffset;
            Offsets.orientation.z = YawOffset;
        }
        else{
            Offsets.position.x = 0;
            Offsets.position.y = 0;
            Offsets.position.z = 0;
            Offsets.orientation.x = 0;
            Offsets.orientation.y = 0;
            Offsets.orientation.z = 0;

        }
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
    cvui::text(frame, 20, 30, state, 0.5, 0xffffff);

}

void CheckMouse(){
    if (cvui::mouse(cvui::CLICK)) {
        //cvui::text(image_copy, 10, 70, "Mouse was clicked!");
        if((cvui::mouse().x>=375)&&((cvui::mouse().y>=10)&&(cvui::mouse().y<=490))) {
            int mouseX = cvui::mouse().x-375;
            int mouseY = cvui::mouse().y-10;
            //if pick up marker exists in this area & is selected and state == 0
            if (state == Picking) {
                printf("Picking Up at %d %d \n", mouseX, mouseY);
                int ID = LocateNearestMarker({(float) mouseX, (float) mouseY});
                if (ID == PlaceID) {
                    state = Wrong;
                } else {
                    PickID = ID;
                    state = Picking;
                }
                checkReady();


            } else if (state == Placing) {

                printf("Placing at %d %d \n", mouseX, mouseY);
                int ID = LocateNearestMarker({(float) mouseX, (float) mouseY});
                if (ID == PickID) {
                    state = Wrong;
                } else {
                    PlaceID = ID;
                    state = Placing;

                }
                checkReady();

            }

        }
    }
}

void checkReady(){
    if (PickID == 2512 | PlaceID == 7320){
        return;
    }
    else{
        state = Ready;
        return;
    }
}


void publishAllTheRos(){

    std_msgs::Int64 msg;
    msg.data = CloseSpeedPercent;
    GripperSpeedPublisher.publish(msg);
    msg.data = ClosePercent;
    GripperClosePercentPublisher.publish(msg);
    msg.data = PickID;
    PickIDPublisher.publish(msg);
    msg.data = PlaceID;
    PlaceIDPublisher.publish(msg);
    std_msgs::String str_msg;
    CommandPublisher.publish(command);
    str_msg.data = GripperState;
    GripperStatePublisher.publish(str_msg);
    OffsetPublisher.publish(Offsets);
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
            it->isVisible = true;
            return;
        }
    }
    ConfirmedIDs.push_back(marker);
}

int LocateNearestMarker(cv::Point2f clickLocation) {
    int nearestID = 65535;
    cv::Point2f NearestMarerLocation = {0,0};
    float disttoNearest = sqrt(pow((NearestMarerLocation.x-clickLocation.x),2)+(pow((NearestMarerLocation.y-clickLocation.y), 2)));
    for (std::vector<Marker>::iterator it = ConfirmedIDs.begin(); it != ConfirmedIDs.end(); it++) {
        if (it->isVisible) {
            float disttoCurrentMarer = sqrt(
                    pow((it->centroid.x - clickLocation.x), 2) + (pow((it->centroid.y - clickLocation.y), 2)));
            if (disttoCurrentMarer < disttoNearest && disttoCurrentMarer < MAXCLICKERROR) {
                disttoNearest = disttoCurrentMarer;
                nearestID = it->ID;
            }
            else {
                //clicked too far away
                state = Wrong;
            }
        }
    }
    printf("%d\n",nearestID );
    return nearestID;
}


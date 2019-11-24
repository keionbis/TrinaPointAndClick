#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>       /* sin */
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
#define PI 3.14159265

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
Marker LocateNearestMarker(cv::Point2f clickLocation);
void initialisePublishers();
void initialiseSubscribers();
void checkReady();
Marker MidpointMarkers(Marker MarkerA, Marker MarkerB);
//GLOBAL VARIABLES

//Marker vectors
static std::vector<Marker> Markers;
static std::vector<Marker> ConfirmedIDs;
//CV Variables
static cv::Mat frame = cv::Mat(600, 1024, CV_8UC3);
static std::string Pick = "Click 'Pick'.";
static std::string Place = "Click 'Place'.";
static std::string Wrong = "No marker near selection. Try clicking a different spot!";
static std::string Picking = "Click a marker to pick up.";
static std::string Placing = "Click a marker to place.";
static std::string MidPoint1 = "Click The first Marker";
static std::string MidPoint2 = "Click The second Marker";

static std::string Doing = "Robot in action now";
static std::string Done = "I did it! Now click 'Pick'.";
static std::string Ready = "Click 'Act' to make the robot do stuff.";
static std::string state = Pick;
static cv::String Act = "Act";
static std_msgs::String command;
static geometry_msgs::Pose Offsets;
static geometry_msgs::PoseStamped PoseStamped;
static bool currentRobotStatus = false;

//Publisher state variables
static bool AuxCameraOpen = false;
static bool ApplyOffsets = false;
static bool LiveOffsets = false;
static bool GripperOpen = false;
//offset and gripper variables
static int XOffset = 0, YOffset = 0, ZOffset = 0; //in mm
static int RollOffset = 0, PitchOffset = 0, YawOffset = 0; //in degrees
static cv::String GripperState = "Open Gripper";
static int ClosePercent = 95, CloseSpeedPercent = 20; //Gripper data

static Marker tmpData;
static int PickID = 2512;
static int PlaceID = 7320;

//ROS Publishers
static ros::Publisher MarkerPosePublisher;
static ros::Publisher GripperSpeedPublisher;
static ros::Publisher GripperClosePercentPublisher;
static ros::Publisher PickIDPublisher;
static ros::Publisher PlaceIDPublisher;
static ros::Publisher CommandPublisher;
static ros::Publisher GripperStatePublisher;
static ros::Publisher OffsetPublisher;
//ROS Subscribers
static ros::Subscriber currentStatusSubscriber;
static cv::String PlaceBtn = "Place On";
cv::Mat camera_matrix, dist_coeffs;
cv::Mat image_copy;
float actual_marker_l = 0.101; // this should be in meters
bool InMidpoint = false;
static Marker ID_1;
static Marker ID_2;

void CurrentStatusCallback(const std_msgs::Bool::ConstPtr& Status){
    printf("lkhsgdbgkas");
    //read in data being published about whether the task is complete or on-going
    currentRobotStatus = Status->data;

    //checks if action was completed
    if (state == Doing){
        if (currentRobotStatus){
            state = Done;
            PickID = 2512;
            PlaceID = 7320;
            InMidpoint = false;
            printf("Done");
        }
    }
}

int main(int argc, char *argv[])
{
    int wait_time = 10;
    cv::Mat image;
    std::ostringstream vector_to_marker;
     ID_1.ID = 7320;
     ID_2.ID = 7320;
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

    currentStatusSubscriber = n.subscribe("CurrentStatus", 1, CurrentStatusCallback);

    //ros::Timer timer1 = n.createTimer(ros::Duration(1), publishAllTheRos);
    cv::VideoCapture in_video;

    in_video.open(0);//Camera index should be a passed parameter

    cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    cv::FileStorage fs("/home/trina/TrinaPointAndClick/src/TrinaPointAndClick/calibration_params.yml", cv::FileStorage::READ); //hard coded calibration file
//  cv::FileStorage fs("../../../src/TrinaPointAndClick/calibration_params.yml", cv::FileStorage::READ); //hard coded calibration file
//    cv::FileStorage fs(argv[2], cv::FileStorage::READ); //parameter passes calibration file

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    std::cout << "camera_matrix\n"
              << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n"
              << dist_coeffs << std::endl;
    // Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
    cvui::init(WINDOW_NAME);


    while (in_video.grab()&& ros::ok()){ //Loop while video exists
        //Start Image Processing
        in_video.retrieve(image);
        
        cv::Point2f center((image.cols-1)/2.0, (image.rows-1)/2.0);
        //cv::Mat rot = cv::getRotationMatrix2D(center, -180, 1.0);

        //warpAffine(image, image, rot, image.size());

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
                if(ConfirmedIDs[x].ID == 999){
                 ConfirmedIDs[x].isVisible = true;
                }
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
                    PoseStamped.pose.orientation.x = rvecs[i][0]+double(RollOffset)*(PI/180);
                    PoseStamped.pose.orientation.y = rvecs[i][1]+double(PitchOffset)*(PI/180);
                    PoseStamped.pose.orientation.z = rvecs[i][2]+double(YawOffset)*(PI/180);
                    PoseStamped.pose.position.x = tvecs[i][0]+double(XOffset)/1000;
                    PoseStamped.pose.position.y = tvecs[i][1]+double(YOffset)/1000;
                    PoseStamped.pose.position.z = tvecs[i][2]+double(ZOffset)/1000;
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
                if(InMidpoint){
                    MidpointMarkers(ID_1, ID_2);
                }
                
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
        if(it->ID == id || id == 999) {
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
            axisPoints.push_back
(cv::Point3f(-half_l, -half_l, 0));
            axisPoints.push_back(cv::Point3f(-half_l, half_l, 0));

            std::vector<cv::Point2f> imagePoints;
            projectPoints(
                    axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints
            );
            cv::Scalar blue (255, 0, 0);//un-picked
            cv::Scalar yellow (0, 255, 255);//picked for pickup
            cv::Scalar teal (255, 225,0 );//picked for place
            cv::Scalar purple (255, 0,255 );//picked Midpoint

            cv::Scalar color = blue;


            if(id == PickID ){
                color = yellow;
            }
            else if(id == PlaceID && !InMidpoint){
                color = teal;
            }
            else if((id == ID_1.ID || id == ID_2.ID)&& InMidpoint){//the id for midpoint
                color = purple;
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
    if (cvui::button(frame, 30, 80,120,40 ,  "Pick")) {
        state = Picking;
    }

    if (cvui::button(frame, 180, 80,120,40 , PlaceBtn)) {
        if(PlaceBtn == "Place On") {
            state = Placing;
            InMidpoint = false;
            PlaceBtn = "Place Midpoint";
        }
        else if(PlaceBtn == "Place Midpoint") {

            state = MidPoint1;
            PlaceBtn = "Place On";

        }
    }

    if (cvui::button(frame, 500, 500,120,40 , Act)) {
        //stop publishers running
        if (Act == "Act"){
            if (state == Ready){
                state = Doing;
                command.data = "act";
                Act = "Pause";
            }
        }
        else if (Act == "Pause"){
            command.data = "cancel";
            Act = "Resume";
        }
        else if (Act == "Resume"){
            command.data = "act";
            Act = "Pause";
        }

    }
    if (cvui::button(frame, 650, 500,120,40 , "Cancel")) {
        //stop publishers running
        state = Pick;
        Act = "Act";
        command.data = "cancel";
    }

    if (cvui::button(frame, 500, 550,120,40 ,  "Reset")) {
        state = Pick;
        Act = "Act";
        PickID = 2512;
        PlaceID = 7320;
        currentRobotStatus = false;
        InMidpoint = false;
        ID_1.ID = 7320;
        ID_2.ID = 7320;
        PlaceBtn = "Place On";
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
        ClosePercent = 95;
        CloseSpeedPercent = 20;
        command.data = "cancel";
    }
    if (cvui::button(frame, 650, 550,120,40 , "&Home")) {
        //tell robot to go to its neutral pose
        command.data = "home";
        Act = "Act";
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
            Offsets.position.x = double(XOffset)/1000;
            Offsets.position.y = double(YOffset)/1000;
            Offsets.position.z = double(ZOffset)/1000;
            Offsets.orientation.x = double(RollOffset)*(PI/180);
            Offsets.orientation.y = double(PitchOffset)*(PI/180);
            Offsets.orientation.z = double(YawOffset)*(PI/180);
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
                    GripperState = "Open Gripper";
                    printf("Closing gripper\n");
                }
                else{
                    GripperState = "Close Gripper";
                    printf("Opening gripper\n");

                }
                GripperOpen =! GripperOpen;
                printf("GripperOpen: %d\n", GripperOpen);

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
                int ID = LocateNearestMarker({(float) mouseX, (float) mouseY}).ID;

                if (ID == PlaceID) {
                    state = Wrong;
                } else {
                    PickID = ID;
                    state = Picking;
                }
                checkReady();


            } else if (state == Placing) {

                printf("Placing at %d %d \n", mouseX, mouseY);
                int ID = LocateNearestMarker({(float) mouseX, (float) mouseY}).ID;
                if (ID == PickID) {
                    state = Wrong;
                } else {
                    PlaceID = ID;
                    state = Placing;

                }
                checkReady();

            }else if (state == MidPoint1 || state == MidPoint2){
                
                if(state == MidPoint1){
                     ID_1 = LocateNearestMarker({(float) mouseX, (float) mouseY});
                     state = MidPoint2;
                     InMidpoint = true;
                }
                else{
                     ID_2 = LocateNearestMarker({(float) mouseX, (float) mouseY});

                    MidpointMarkers(ID_1, ID_2);
                    state = MidPoint1;
                    PlaceID = 999;
                    checkReady();
                }
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
    CommandPublisher.publish(command);
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

Marker LocateNearestMarker(cv::Point2f clickLocation) {
    Marker nearestID;
    cv::Point2f NearestMarerLocation = {0,0};
    float disttoNearest = sqrt(pow((NearestMarerLocation.x-clickLocation.x),2)+(pow((NearestMarerLocation.y-clickLocation.y), 2)));
    for (std::vector<Marker>::iterator it = ConfirmedIDs.begin(); it != ConfirmedIDs.end(); it++) {
        if (it->isVisible) {
            float disttoCurrentMarer = sqrt(
                    pow((it->centroid.x - clickLocation.x), 2) + (pow((it->centroid.y - clickLocation.y), 2)));
            if (disttoCurrentMarer < disttoNearest && disttoCurrentMarer < MAXCLICKERROR) {
                disttoNearest = disttoCurrentMarer;
                nearestID = {it->ID, it->timesSeen,
                             {it->location[0], it->location[1], it->location[2], it->location[3]},
                             it->centroid, it->isVisible};
            }
            else {
                //clicked too far away
                state = Wrong;
            }
        }
    }
    printf("%d\n",nearestID.ID );
    return nearestID;
}

Marker MidpointMarkers(Marker MarkerA, Marker MarkerB){
    if(MarkerA.centroid.x == MarkerB.centroid.x || MarkerA.centroid.y == MarkerB.centroid.y){

    }
    else{
        std::vector<cv::Vec3d> rvecs, tvecs;
               
        float x = ((MarkerA.centroid.x + MarkerB.centroid.x) / 2);
        float y = ((MarkerA.centroid.y + MarkerB.centroid.y) / 2);

        cv::Point2f _MidpointCentroid;
        _MidpointCentroid.x = x;
        _MidpointCentroid.y = y;
        
        cv::circle( image_copy,
         _MidpointCentroid,
         17,
         cv::Scalar( 255,255,0),
         4,
         8 );
        
        std::vector<std::vector<cv::Point2f>> corners;
        corners.push_back({MarkerA.location[0], MarkerA.location[1],MarkerA.location[2], MarkerA.location[3]});
        corners.push_back({MarkerB.location[0], MarkerB.location[1],MarkerB.location[2], MarkerB.location[3]});
        cv::aruco::estimatePoseSingleMarkers(
                    corners, actual_marker_l, camera_matrix, dist_coeffs,
                    rvecs, tvecs
            );
        
       
       
       /*
        PoseStamped.header.stamp = ros::Time::now();
        PoseStamped.header.frame_id = std::to_string(999);
        PoseStamped.pose.orientation.w = 0;
        PoseStamped.pose.orientation.x = (rvecs[0][0]+rvecs[1][0])/2;
        PoseStamped.pose.orientation.y = (rvecs[0][1]+rvecs[1][1])/2;
        PoseStamped.pose.orientation.z = (rvecs[0][2]+rvecs[1][2])/2;
        PoseStamped.pose.position.x = (tvecs[0][0]+tvecs[1][0])/2;
        PoseStamped.pose.position.y = (tvecs[0][1]+tvecs[1][1])/2;
        PoseStamped.pose.position.z = (tvecs[0][2]+tvecs[1][2])/2;
        MarkerPosePublisher.publish(PoseStamped);
        */            
        


    }
}

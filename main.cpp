#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW_NAME	"Autonomous Grasping"

void drawCubeWireFrame(
        cv::InputOutputArray image, cv::InputArray cameraMatrix,
        cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
        float l
);


int main(int argc, const char *argv[])
{
    int state = 0;
    int wait_time = 10;
    float actual_marker_l = 0.101; // this should be in meters
    cv::Mat frame = cv::Mat(600, 1024, CV_8UC3);

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

    std::cout << "camera_matrix\n"
              << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n"
              << dist_coeffs << std::endl;

    int image_width = in_video.get(CV_CAP_PROP_FRAME_WIDTH);
    int image_height = in_video.get(CV_CAP_PROP_FRAME_HEIGHT);
    int fps = 60;


//    cv::Mat image = cv::Mat(980, 600, CV_8UC3);

    // Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
    cvui::init(WINDOW_NAME);

    // Rectangle to be rendered according to mouse interactions.
    cv::Rect rectangle(0, 0, 0, 0);

    while (in_video.grab()){
        in_video.retrieve(image);

        image.copyTo(image_copy);
        frame = cv::Scalar(30, 40, 40);
        if(state == 0) {
            cvui::text(frame, 20, 30, "Please select marker to pick up ", 0.5, 0xffffff);
        }
        else if(state == 1){
            cvui::text(frame, 20, 30, "Please select placement marker ", 0.5, 0xffffff);
        }
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // Fill the image with a nice color

        // Did any mouse button go down?

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

            //if state == 1 && place marker exists here



        }
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


                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x: " << std::setw(8) << tvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                            cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cvScalar(0, 252, 124), 1, CV_AA);
            }
        }
        cvui::image(frame, 375, 10, image_copy);
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


        cvui::rect(frame, 10, 200, 300, 300, 0xffffff);
        //x pos
        cvui::rect(frame, 50, 230, 60, 20, 0xffffff);
        cvui::printf(frame, 55, 235, "%d", 100);
        cvui::rect(frame, 50, 270, 60, 20, 0xffffff);
        cvui::printf(frame, 55, 275, "%d", 100);
        cvui::rect(frame, 50, 310, 60, 20, 0xffffff);
        cvui::printf(frame, 55, 315, "%d", 100);

        cvui::update();

        // Show everything on the screen
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
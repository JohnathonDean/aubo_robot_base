#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat inImage;
    try
    {
        // ROS_INFO("image encodings %s",msg->encoding.c_str());
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        inImage = cv_ptr->image;
        int high = inImage.rows;
        int width = inImage.cols;
        //draw lines of different colours
        cv::line(inImage,Point(1, high/2),Point(width, high/2),Scalar(255,0,0,255),1,CV_AA);
        cv::line(inImage,Point(width/2,1),Point(width/2, high),Scalar(0,255,0,255),1,CV_AA);
        //draw circle of different colours
        cv::circle(inImage, Point(width/2, high/2), 20, Scalar(0,0,255,255), 1, CV_AA);
        //draw rectangle
        cv::rectangle(inImage,Point(-100+width/2, -100+high/2),Point(100+width/2, 100+high/2),Scalar(0,255,0,255),1,CV_AA);
        cv::imshow("view", inImage);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_draw");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);

    ros::spin();
    cv::destroyWindow("view");
}



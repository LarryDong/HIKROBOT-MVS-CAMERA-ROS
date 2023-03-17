
#include <fstream>
#include <mutex>
#include <iostream>
#include <string>
#include <signal.h>
#include <vector>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "hikrobot_camera.hpp"


using namespace std;
using namespace cv;


void exit_handler(int sig_num){
	printf("SIGNAL received: num =%d\n", sig_num);
	if (sig_num == 1 || sig_num == 2 || sig_num == 3 || sig_num == 9 || sig_num == 15){
		exit(0);
	}
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle nh;
    
    bool show_image = false;
    ros::param::get("~show_image", show_image);
    
    sensor_msgs::Image image_msg;
    // sensor_msgs::CameraInfo camera_info_msg;                // Not used.
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/image", 10);
    
    camera::Camera MVS_cap(nh);
    ros::Rate r(100);
    ROS_INFO("Begin to publish image...");
    while (ros::ok()){
        cv::Mat src;
        MVS_cap.ReadImg(src);
        if (src.empty())
            continue;
        cv_ptr->image = src;
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = "hikrobot_camera";
        image_pub.publish(image_msg);
        // camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    // camera_info_msg.header.stamp = image_msg.header.stamp;
        if(show_image){
            imshow("src", src);
            int c = waitKey(5);
            if (c == 'q')
                std::abort();
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}


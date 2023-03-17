
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

bool g_begin = false, g_shutdown = false; // stop the node
ros::Time g_beginTime;
vector<sensor_msgs::Image> g_vImageMsg;
mutex m_vImageMsg;


void exit_handler(int sig_num){
	printf("SIGNAL received: num =%d\n", sig_num);
	if (sig_num == 1 || sig_num == 2 || sig_num == 3 || sig_num == 9 || sig_num == 15){
		exit(0);
	}
}

void CLIHandler(const std_msgs::String s){
    if(s.data == "b"){
        ROS_WARN("Begin recording image...");
        g_beginTime = ros::Time::now();     // maybe not the same as event's, but just ignore it.
        g_begin = true;
        // g_vImageMsg.clear();
    }
    if(s.data == "s"){
        g_shutdown = true;
    }
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 

    image_transport::ImageTransport it(hikrobot_camera);
    image_transport::Publisher image_pub = it.advertise("/image", 10);
    ros::Subscriber subString = hikrobot_camera.subscribe<std_msgs::String>("/cmd", 1, CLIHandler);

    
    ros::Rate loop_rate(100);

    cv::Mat src;
    int counter = 0;
    while (!g_shutdown){

        MVS_cap.ReadImg(src);
        if (src.empty())
            continue;

        cv_ptr->image = src;
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = "hikrobot_camera";
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg);

        std_msgs::Header hd;
        hd.stamp = ros::Time::now();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(hd, "bgr8", src).toImageMsg();
        


        if (g_begin){
            g_vImageMsg.push_back(*msg);
        }

        // imshow("src", src);
        // int c = waitKey(1);
        // if (c == 'q')
        //     return -1;
        // else if(c=='s'){
        //     cout << "Saving no. " << counter << " image. " << endl;
        //     imwrite("/home/larrydong/Desktop/" + to_string(counter++) + ".bmp", src);
        // }
        loop_rate.sleep();
        ros::spinOnce();
    }

    ROS_WARN("Save image begin...");
    m_vImageMsg.lock();
    string image_dir = "/home/larrydong/output/image";
    ROS_INFO_STREAM("Save image to dir: " << image_dir);
    ofstream of(image_dir + "/image_ts.txt");
    if (!of.is_open()){
        ROS_ERROR_STREAM("Cannot save events to: " << image_dir);
        return -1;
    }
    for (int i = 0; i < g_vImageMsg.size(); ++i){
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(g_vImageMsg[i], "bgr8");
        imwrite(image_dir + "/" + to_string(i)  + ".bmp", cv_ptr->image);
        of << i << " " << (int)((cv_ptr->header.stamp - g_beginTime).toSec() * 1e6) << endl; // save us from 0
    }
    of.close();
    ROS_WARN("Save data end.");

    return 0;
}


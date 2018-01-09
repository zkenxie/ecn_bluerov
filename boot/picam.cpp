#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std; 
 
int main ( int argc,char **argv ) {
   
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;
    
    //set camera params
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //Open camera
    cout<<"Opening Camera..."<<endl;
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    //Start capture
    //cout<<"Capturing "<<nCount<<" frames ...."<<endl;
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    ros::Rate loop(20);

    while(ros::ok())
    {
        Camera.grab();
        Camera.retrieve ( image);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	pub.publish(msg);
	ros::spinOnce();
	loop.sleep();
    }
    cout<<"Stop camera..."<<endl;
    Camera.release();
}

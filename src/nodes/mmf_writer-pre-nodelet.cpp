#include <ros/ros.h>
//#include <nodelet/loader.h>
//#include <mmf_writer/advertisement_checker.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "mmf_writer/StaticBackgroundCompressor.h"
#include "mmf_writer/ThreadedLinearStackCompressor.h"
#include "mmf_writer/mmfWriter.h"
#include "std_msgs/Bool.h"



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_ERROR("We got image!");
}


class tempClass
{
public:
	ros::NodeHandle nh_ ;
	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber sub_;
	image_transport::Publisher pub_;
	//tf::TransformListener tf_listener_;
	//image_geometry::PinholeCameraModel cam_model_;


	tempClass() : it_(nh_){
		std::string image_topic = nh_.resolveName("image_raw");
		sub_ = it_.subscribeCamera(image_topic, 1, &tempClass::imageCb, this);

	}

	void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
	                  const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		std::cout<< "yep" ;

	}
};

//void recordingCallback(const std_msgs::Bool &)
//{
//	// This only gets triggered w/ a new message, we still need to check positive or negative :)
//
//	ROS_ERROR("We are checking back the call back!");
//}


int main(int argc, char **argv)
{

	StackReader sr("/home/bruno/teststack.mmf");
//	StackReader sr("/home/bruno/valid.mmf");

//	std::string ya = sr.diagnostics();
//	ROS_WARN_STREAM(ya);
//	sr.playMovie();
	int keyframeInt = sr.getKeyFrameInterval();

	// This is part of cvTester
//    ROS_ERROR_STREAM("size of Ipl Image is: " << sizeof(IplImage) << "  win32 size = 112");
//    ROS_ERROR_STREAM ("size of int is: " << sizeof(int) << "  win32 size = 4" );
//    ROS_ERROR_STREAM ("size of char is: " << sizeof(char) );
//    ROS_ERROR_STREAM ("size of char * is: " << sizeof(char *) );
//    ROS_ERROR_STREAM("size of _iplROI * is: " << sizeof(_IplROI *) );
//    ROS_ERROR_STREAM("size of _IplImage * is: " << sizeof(_IplImage *) );
//    ROS_ERROR_STREAM("size of _IplTileInfo * is: " << sizeof(_IplTileInfo *) );
//
//    ROS_ERROR_STREAM ( "size of pointer is: " << sizeof (void *) << "  win32 size = 4" );
//    ROS_ERROR_STREAM ( "20*sizeof(int) + 6*sizeof(pointer) + 8 (2 char arrays[4]) = " << 20*sizeof(int) + 6 * sizeof (void *) + 8 );
//    ROS_ERROR_STREAM ( "sizeof (CvRect) = " << sizeof(CvRect) << "  win32 size = 16" );
//    ROS_ERROR_STREAM ( "sizeof (double) = " << sizeof(double) << "  win32 size = 8" );
//    //you can comment out the next 4 lines if you don't want to link to the cv libraries
//    IplImage *im = cvCreateImage(cvSize(1024,1024), IPL_DEPTH_8U, 1);
//    ROS_ERROR_STREAM ( "im->nsize = " << im->nSize );
//    ROS_ERROR_STREAM ( "*(int *) im = " << *(int *) im );
//    cvReleaseImage(&im);
//    return 0;


	ros::init(argc, argv, "mmf_writer");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);


  mmfWriter myMMF ;
  myMMF.lsc.setThresholds(0, 5, 2, 3);
  // This should set keyframes to 10, but it doesn't seem to work that way?...
  myMMF.lsc.setIntervals(120, 1);
  myMMF.lsc.setFrameRate(30);
//  myMMF.lsc.frameRate = 30; // not sure about the whole relationship between frame rate and intervals?...
  myMMF.lsc.setOutputFileName("/home/bruno/teststack.mmf");
  myMMF.framesToRecord = 2000;
  //  myMMF.startRecording(1000);

//  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::Subscriber subRecord = nh.subscribe("recording/recording", 1, &mmfWriter::recordingCallback, &myMMF);

//  ros::Subscriber subNumFrames = nh.subscribe("recording/framesToRecord", 1, startRecordingCallBack);
//  ros::Subscriber sub1 = nh.subscribe("camera/image_raw", 1000, &mmfWriter::imageCallback, &myMMF);
  // PG camera
  ros::Subscriber sub1 = nh.subscribe("camera/image_mono", 1000, &mmfWriter::imageCallback, &myMMF);

  //  image_transport::Subscriber sub = it.subscribe("camera/image", 1, &mmfWriter::imageCallback, &myMMF);

  //tempClass tempy;
  ros::spin();
  // Shared parameters to be propagated to nodelet private namespaces


  return 0;
}

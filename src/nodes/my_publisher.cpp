#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "mmf_writer/StackReader.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	//image_transport::Publisher image_pub_ = it.advertise("camera/image_raw", 1);
//	image_transport::CameraPublisher cam_pub_ = it.advertiseCamera("camera/image_raw", 1);
	image_transport::CameraPublisher cam_pub_ = it.advertiseCamera("image_raw", 1);

	//cam_pub_.
	//ros::Publisher pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
	//  sensor_msgs::CameraInfo cam_info;

	// get current CameraInfo data
	//      sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo);
	//    sensor_msgs::CameraInfo ci;

	std::string fnameBase = "/home/bruno/images/20141026_154700@FCF_attP2_1500062@UAS_TNT_2_0003@t10@n#n#n#n@30@" ;
	std::string mmfFileName = "/home/bruno/valid.mmf";
	mmfFileName = "/home/bruno/136_saved.mmf";
	cv::Mat image;
	sensor_msgs::ImagePtr img_msg;
	sensor_msgs::CameraInfoPtr ci (new sensor_msgs::CameraInfo());
	ros::Rate loop_rate(30);
	int i = 0;
	int framesToLoad = 50 ;
	std::vector<cv::Mat> matVector ;
	StackReader sr(mmfFileName.c_str());
	IplImage * ipl = NULL;

	//////// ATTENTION! CHANGE HERE
	int whatToLoad = 2 ; // 0, tifs on file; 1, load mmf frames to memory, 2 load from mmf and publish them

	switch (whatToLoad) {
	case 0:
	{
		ROS_INFO("Loading up files into a std::vector");
		for (int f = 0 ; f< framesToLoad ; f++) {
			std::ostringstream ss;
			ss << fnameBase ;
			char fnumber[7];
			snprintf(fnumber, sizeof(fnumber), "%06d", f ) ;

			ss << fnumber << ".tif";
			std::string fnameToOpen = ss.str();

			//ROS_INFO(fnameToOpen.c_str());
			matVector.push_back(cv::imread(ss.str(),CV_LOAD_IMAGE_COLOR));
			//	  image = cv::imread(ss.str(),CV_LOAD_IMAGE_COLOR);
		}
		ROS_INFO("done load files");
		break;
	}

	case 1: // LOAD MMF TO MEMORY
	{
		ROS_INFO("Loading up frames from mmf file into a std::vector");

		//		sr.playMovie(1,100,20,"temp",true);
		//		std::string ya = sr.diagnostics();
		//	std::ostringstream os ;
		int maxFramesToLoad ;
		if (framesToLoad >  sr.getTotalFrames()-1) {
			maxFramesToLoad = sr.getTotalFrames()-1 ;
		} else {
			maxFramesToLoad = framesToLoad;
		}
		//	    cvNamedWindow("temp", 0);

		for (int f=0; f < maxFramesToLoad ; f++ ) {

			sr.getFrame(f, &ipl);
			//cv::Mat toStore = cv::cvarrToMat(ipl);
			cv::Mat toStore(ipl,true);
			//cvShowImage("temp",ipl);
			//cvWaitKey(5);
			matVector.push_back(toStore.clone());
			//const ImageMetaData * md = sr.getMetaData(1);
			//			std::map<std::string, double> gimme = md->getFieldNamesAndValues();
			//			md = sr.getMetaData(f+1);
			//			gimme = md->getFieldNamesAndValues();
			//			os << "bufnum: " << gimme["bufnum"] << ", bufnum_camera: " << gimme["bufnum_camera"] << ", bufnum_time: " << gimme["bufnum_time"] << std::endl ;

		}
		cvReleaseImage(&ipl);
		break;
	case 2: {
		//StackReader sr(mmfFileName.c_str());
		// by default we load the whole file
		framesToLoad = sr.getTotalFrames()-1;
		// either we load what we wanted or smaller
		if (framesToLoad >  sr.getTotalFrames()-1) {
			framesToLoad = sr.getTotalFrames()-1 ;
		}

	}
	}
	}


	while (nh.ok()) {
		if (i > framesToLoad) {
			i = 0;
		}


		switch (whatToLoad) {
		case 0 or 1:
		{
			if (!matVector[i].empty()) {
				image = matVector[i];
				img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", matVector[i]).toImageMsg();
				ci.reset(new sensor_msgs::CameraInfo());
				ci->height = image.rows;
				ci->width = image.cols;
				img_msg->header.stamp = ros::Time::now();
				ci->header.stamp = img_msg->header.stamp;
				//cam_info.header.stamp = ros::Time::now();
				//pub_cam_info.publish(cam_info);
				//			  pub.publish(image);
				cam_pub_.publish(img_msg,ci);
				//image_pub_.publish(image);
				//			  image_pub_(ci);
				//				  cv::waitKey(1);
			}

			break;
		}
		case 2:
		{
				sr.getFrame(i, &ipl);
				//cv::Mat toStore = cv::cvarrToMat(ipl);
				cv::Mat toStore(ipl,true);
				img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", toStore).toImageMsg();
				ci.reset(new sensor_msgs::CameraInfo());
				ci->height = image.rows;
				ci->width = image.cols;
				img_msg->header.stamp = ros::Time::now();
				ci->header.stamp = img_msg->header.stamp;
				cam_pub_.publish(img_msg,ci);

			}



		}


		//		  ROS_INFO("%s", fnameToOpen.c_str());
		ros::spinOnce();
		loop_rate.sleep();
		i++ ;
		}

		//cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

	}


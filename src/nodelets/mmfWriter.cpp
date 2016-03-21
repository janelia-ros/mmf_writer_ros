
#include "mmf_writer/mmfWriter.h"
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>
#include <fstream>

static const std::string OPENCV_WINDOW = "Image window";

void mmfWriter::recordingCallback(const std_msgs::Bool recordMsg) {
	// If we should record and are not recording, we start a new one :)
	bool ya = recordMsg.data;
	if (recordMsg.data && recording == 0 ) {
		startRecording();
		return;
	}
	// This means we were recording
	if (recordMsg.data == 0 && recording == 1 ) {
		lsc.stopRecording();
		return;
	}

}

mmfWriter::mmfWriter() {
	//ThreadedLinearStackCompressor lsc ; // we initialize
	framesToRecord = 0 ;
	framesRecorded = 0 ;
	recording = false;
	cv::namedWindow(OPENCV_WINDOW);
	prevFrameSeq = 0 ;
}

mmfWriter::~mmfWriter() {
	cv::destroyWindow(OPENCV_WINDOW);
}

void mmfWriter::startRecording() {
	//framesToRecord = framesToRecord ;
	framesRecorded = 0 ;
	recording = true;
	lsc.startRecording(framesToRecord);

}

void mmfWriter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// We will need to update this to not callback unless we are recording :)
	if (recording) {

	if (framesRecorded <= framesToRecord ) {

	//ROS_ERROR("We got image all right !");
	BlankMetaData *bmd = new BlankMetaData;
	 cv_bridge::CvImagePtr cv_ptr;
	 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	 cv::Mat yep = cv_ptr->image ;
	 //auto ya =  cv_ptr->header;
	 int frameNum = cv_ptr->header.seq;
	 int frameDiff = frameNum - prevFrameSeq;
	 prevFrameSeq = frameNum ;

	 if (frameDiff > 1) {
		 std::ostringstream os;
		 os << "Skipped " << frameDiff << " frames!" ;
		 ROS_INFO(os.str().c_str());
	 }

	 cv::Mat eightBit ;
	 yep.convertTo(eightBit,CV_8UC1);
	 IplImage *im = new IplImage(eightBit);
	 IplImage* iplImage;

	 //iplImage = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
//	 IplImage stub, *dst_img;
//	 CvMat* mat = cvCreateMatHeader(1000, 1000, CV_8UC1);
//	 dst_img = cvGetImage(&mat, &stub);
//	 IplImage *im = new IplImage(yep);

	lsc.newFrame(im,bmd);
	//lsc.addFrameToStack()
	framesRecorded ++ ;


	} else {
		lsc.stopRecording();
		lsc.closeOutputFile();
		recording = false;
		ROS_WARN("We stopped recording!");

//		StackReader sr("/home/bruno/teststack.mmf");
//		std::string ya = sr.diagnostics();
		//ROS_ERROR(sr.diagnostics());
		 //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		 //cv::waitKey(3);

	}

	}

}

void mmfWriter::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg) {

}

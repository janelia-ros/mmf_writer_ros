//#ifndef MMF_WRITER_NODELET
//#define MMF_WRITER_NODELET

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <mmf_writer/ProcessImageConfig.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>

#include "std_msgs/Empty.h"
# include <std_srvs/Empty.h>

#include "mmf_writer/RecordingStatus.h"
#include "mmf_writer/mmfWriter.h"
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/Bool.h"

#include "mmf_writer/mmfWriter.h"
#include "mmf_writer/NameValueMetaData.h"
#include <wfov_camera_msgs/WFOVImage.h>

#include <actionlib/server/simple_action_server.h>
#include <mmf_writer/RecordAction.h>
#include <mmf_writer/RecordingActionServer.h>

static const std::string OPENCV_WINDOW = "Image window";

namespace mmf_writer {


class ProcessCameraFrame : public nodelet::Nodelet
{
public:
	// ROS communication
	boost::shared_ptr<image_transport::ImageTransport> it_in_,it_out_;

	image_transport::CameraSubscriber cam_sub_ ; //, cam_wfov_sub_;
	ros::Subscriber cam_wfov_sub_;

	RecordingActionServer * actionServer ;

	// action server stuff
	void goalCB() ;

	int queue_size_;

	boost::mutex connect_mutex_;

	ros::Publisher recordingStatus_pub_;
	ros::ServiceServer refreshParam_srv_;

	bool refreshParamCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);


	// background image
	ros::Subscriber recording_sub_, start_sub_, stop_sub_;
	std::string background_image_path_;
	cv::Mat image_background_;
	bool save_background_image_;

	// dynamic reconfigure
	boost::recursive_mutex config_mutex_;
	typedef mmf_writer::ProcessImageConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	Config config_;

	virtual void onInit();

	void callback(const std_msgs::Bool str);
	void recordCb() ; // This one gets called to start recording :)
	void connectCb(); // This was called when clients subscribed to its image...

	void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg);

	void imageWFOVCb(const wfov_camera_msgs::WFOVImageConstPtr& wfovImg) ;

	void configCb(Config &config,uint32_t level);

	void saveBackgroundImageCb(const std_msgs::Empty::ConstPtr& msg);
	void recordingCb(const std_msgs::Bool& inputMsg);
//	void startRecordingCb(const std_msgs::Bool& inputMsg);
//	void stopRecordingCb(const std_msgs::Bool& inputMsg);
	void startRecordingCb(const std_msgs::EmptyConstPtr msg);
	void stopRecordingCb(const std_msgs::EmptyConstPtr msg);


private:

    // Action servers
//    recordServer yep ;

	// variables
	bool recording_ , firstFrame_, instantiated_;
	int framesRecorded_, framesToRecord_, keyframeInterval_ ;
	int secondsToRecord_, secondsRecorded_ ;

	int bufnum_ ;
	std::string fname_, path_ ;
	ThreadedLinearStackCompressor lsc;
	int prevFrameSeq_;
	ros::Duration frameTimeStamp_, elapsedRecordingTime_ ;
	ros::Time initialTimeStamp_, recordingStartTime_ ;
	long initialCameraSeq_ ;
	// fncs
	void startRecording();
	void stopRecording();
	void setParameters();
	void connectCamera();
	void disconnectCamera();
	};


//PLUGINLIB_EXPORT_CLASS( mmf_writer::ProcessCameraFrame,nodelet::Nodelet)

}


//#endif



//
//void mmfWriter::recordingCallback(const std_msgs::Bool recordMsg) {
//	// If we should record and are not recording, we start a new one :)
//	bool ya = recordMsg.data;
//	if (recordMsg.data && recording == 0 ) {
//		startRecording();
//		return;
//	}
//	// This means we were recording
//	if (recordMsg.data == 0 && recording == 1 ) {
//		lsc.stopRecording();
//		return;
//	}
//
//}
//
//mmfWriter::mmfWriter() {
//	//ThreadedLinearStackCompressor lsc ; // we initialize
//	framesToRecord = 0 ;
//	framesRecorded = 0 ;
//	recording = false;
//	cv::namedWindow(OPENCV_WINDOW);
//	prevFrameSeq = 0 ;
//}
//
//mmfWriter::~mmfWriter() {
//	cv::destroyWindow(OPENCV_WINDOW);
//}
//
//void mmfWriter::startRecording() {
//	//framesToRecord = framesToRecord ;
//	framesRecorded = 0 ;
//	recording = true;
//	lsc.startRecording(framesToRecord);
//
//}
//
//void mmfWriter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//	// We will need to update this to not callback unless we are recording :)
//	if (recording) {
//
//	if (framesRecorded <= framesToRecord ) {
//
//	//ROS_ERROR("We got image all right !");
//	BlankMetaData *bmd = new BlankMetaData;
//	 cv_bridge::CvImagePtr cv_ptr;
//	 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//	 cv::Mat yep = cv_ptr->image ;
//	 //auto ya =  cv_ptr->header;
//	 int frameNum = cv_ptr->header.seq;
//	 int frameDiff = frameNum - prevFrameSeq;
//	 prevFrameSeq = frameNum ;
//
//	 if (frameDiff > 1) {
//		 std::ostringstream os;
//		 os << "Skipped " << frameDiff << " frames!" ;
//		 ROS_INFO(os.str().c_str());
//	 }
//
//	 cv::Mat eightBit ;
//	 yep.convertTo(eightBit,CV_8UC1);
//	 IplImage *im = new IplImage(eightBit);
//	 IplImage* iplImage;
//
//	 //iplImage = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 1);
////	 IplImage stub, *dst_img;
////	 CvMat* mat = cvCreateMatHeader(1000, 1000, CV_8UC1);
////	 dst_img = cvGetImage(&mat, &stub);
////	 IplImage *im = new IplImage(yep);
//
//	lsc.newFrame(im,bmd);
//	//lsc.addFrameToStack()
//	framesRecorded ++ ;
//
//
//	} else {
//		lsc.stopRecording();
//		lsc.closeOutputFile();
//		recording = false;
//		ROS_WARN("We stopped recording!");
//
////		StackReader sr("/home/bruno/teststack.mmf");
////		std::string ya = sr.diagnostics();
//		//ROS_ERROR(sr.diagnostics());
//		 //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//		 //cv::waitKey(3);
//
//	}
//
//	}
//
//}
//
//void mmfWriter::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
//		const sensor_msgs::CameraInfoConstPtr& info_msg) {
//
//}

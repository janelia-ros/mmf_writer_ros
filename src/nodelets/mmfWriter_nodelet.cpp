#include "mmfWriter_nodelet.h"

// register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( mmf_writer::ProcessCameraFrame,nodelet::Nodelet)
// deprecated
//PLUGINLIB_DECLARE_CLASS( mmf_writer, ProcessCameraFrame,mmf_writer::ProcessCameraFrame, nodelet::Nodelet)

#include "std_msgs/Bool.h"


namespace mmf_writer {


void ProcessCameraFrame::onInit()
{
	ROS_INFO("We initialize the ProcessCameraFrame class!");
	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();
	it_in_.reset(new image_transport::ImageTransport(nh));
//	ros::NodeHandle nh_in (nh,"camera");
//	it_in_.reset(new image_transport::ImageTransport(nh_in));

	//ros::NodeHandle nh_out(nh,"blob_out");
	//it_out_.reset(new image_transport::ImageTransport(nh_out));

	// We init variables:
	recording_ = false;
	instantiated_ = true;

	// read parameters, default to 500 :)
	private_nh.param("queue_size",queue_size_,500);
	private_nh.param("frames_to_record",framesToRecord_,50);
	private_nh.param<std::string>("filename",fname_,"ros_nodelet.mmf");
	private_nh.param<std::string>("path",path_,"/home/bruno/recordings/");
	private_nh.param("keyframe_interval",keyframeInterval_,12);
	private_nh.param("seconds_to_record",secondsToRecord_,5);

	// set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(config_mutex_,private_nh));
	ReconfigureServer::CallbackType f = boost::bind(&ProcessCameraFrame::configCb,this,_1,_2);
	reconfigure_server_->setCallback(f);

	boost::lock_guard<boost::mutex> lock(connect_mutex_);

	// start saving a movie
	//private_nh.subscribe("recording",1,&ProcessCameraFrame::recordingCb,this);
	start_sub_ = private_nh.subscribe("start",1,&ProcessCameraFrame::startRecordingCb,this);
	stop_sub_ = private_nh.subscribe("stop",1,&ProcessCameraFrame::stopRecordingCb,this);

	//cam_wfov_sub_ = nh.subscribe("image",1,&ProcessCameraFrame::imageWFOVCb,this);

//	ros::NodeHandle& private_nh_mt = getMTPrivateNodeHandle();
	private_nh.param<std::string>("background_image_path",background_image_path_,"background.png");
	save_background_image_ = false;

	refreshParam_srv_ = nh.advertiseService("refreshParams", &ProcessCameraFrame::refreshParamCallback, this);

//	ros::SubscriberStatusCallback connect_cb_blobs = boost::bind(&ProcessImageNodelet::connectCb,this);
	recordingStatus_pub_ = nh.advertise<RecordingStatus>("recordingStatus",2);

	actionServer = new RecordingActionServer("mmf_recorder",nh);
//	actionServer-> as_.registerGoalCallback(boost::bind(&ProcessCameraFrame::goalCB, this)) ;
//	actionServer->as_.registerPreemptCallback(boost::bind(&RecordingActionServer::preemptCB, this)) ;
//	as_.registerPreemptCallback(boost::bind(&RecordingActionServer::preemptCB, this));

}

void ProcessCameraFrame::goalCB()
{
	ROS_INFO("We set a new goal inside mmfwriter_nodelet");
	// reset helper variables
//	goal_ = as_.acceptNewGoal()->recorder_settings;
}

bool ProcessCameraFrame::refreshParamCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtlesim.");
  // We will refresh parameters here!
  return true;
}


void ProcessCameraFrame::startRecording() {
	framesRecorded_ = 0 ;
	lastFrame_ = false;
	bufnum_time_ = 0;
	lostFrames_ = 0;
	connectCamera() ;

	Config config;
	{
		boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
		config = config_;
	}

	/* virtual inline void setThresholds(int threshBelowBackground, int threshAboveBackground, int smallDimMinSize, int lgDimMinSize) {
	 *
	 * threshBelowBackground: pixel values less than the background value must be this much below the background to count
	 * threshAboveBackground: pixel values greater than the background value must be this much above the background to count
	 * smallDimMinSize (s), lgDimMinSize (l): regions containing non-background pixels must be at least these dimensions (sxl or lxs)
	 */
	lsc.setThresholds(0, 5, 2, 3); // sticking w/ defaults for now
	// This should set keyframes to 10, but it doesn't seem to work that way?...
	lsc.setIntervals(config.keyframe_interval, 1);
	lsc.setFrameRate(config.frame_rate);
	//  myMMF.lsc.frameRate = 30; // not sure about the whole relationship between frame rate and intervals?...
	std::ostringstream fp ;
	fp << config.path << config.filename << ".mmf";
	std::string ye = fp.str().c_str();
	lsc.setOutputFileName(fp.str().c_str());
//	lsc.setOutputFileName("/home/bruno/teststack.mmf");
	framesToRecord_ = config.frames_to_record;
//	lsc.startRecording(framesToRecord_);
	// We'll use time instead of frames, so we set a ridiculous high value
//	secondsToRecord_ = config.secs_to_record;
	//secondsToRecord_ = 5;

	lsc.startRecording(999999);
	recording_ = true;

	firstFrame_ = true;
	ROS_INFO("We started recording a new file!");
	recordingStartTime_ = ros::Time::now();
	secondsRecorded_ = 0 ;
}

void ProcessCameraFrame::connectCamera() {
	boost::lock_guard<boost::mutex> lock(connect_mutex_);
	if (!cam_wfov_sub_)
	{
		// read background image if one exists
		//image_background_ = cv::imread(background_image_path_,CV_LOAD_IMAGE_GRAYSCALE);

		ROS_INFO("We subscribe to the camera!");
		//image_transport::TransportHints hints("raw",ros::TransportHints(),getPrivateNodeHandle());
//		ros::NodeHandle& private_nh = getPrivateNodeHandle();
		//cam_wfov_sub_ = private_nh.subscribe("image",queue_size_,&ProcessCameraFrame::imageWFOVCb,this);
		ros::NodeHandle& nh = getNodeHandle();
		cam_wfov_sub_ = nh.subscribe("image",queue_size_,&ProcessCameraFrame::imageWFOVCb,this);

//		image_transport::TransportHints hints("raw",ros::TransportHints(),getPrivateNodeHandle());
//		cam_sub_ = it_in_->subscribeCamera("image_raw",queue_size_,&ProcessCameraFrame::imageCb,this,hints);

	}
}

void ProcessCameraFrame::imageWFOVCb(const wfov_camera_msgs::WFOVImageConstPtr& wfovImg) {
	//ROS_INFO("We enter imageWFOVCb");

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(wfovImg->image);

	sensor_msgs::Image image_msg = wfovImg->image ;
	sensor_msgs::CameraInfo info_msg = wfovImg->info ;

	if (recording_) {
		// We need to change this for time based, not frame based
		elapsedRecordingTime_ = ros::Time().now() - recordingStartTime_ ;
		if (secondsRecorded_ != elapsedRecordingTime_.sec) {
			secondsRecorded_ = elapsedRecordingTime_.sec;
			std::ostringstream ts;
			ts << "Elapsed Time: " << elapsedRecordingTime_.sec << " sec";
			ROS_INFO(ts.str().c_str());
		}

		if ( bufnum_time_ - 0.1 > elapsedRecordingTime_.sec  ) {
			// a line worth days of work...
			int headerFrameNum = wfovImg->frame_counter;
			int frameDiff = headerFrameNum - prevFrameSeq_;
			prevFrameSeq_ = headerFrameNum ;

			if (frameDiff > 1) {
				std::ostringstream os;
				os << "Skipped " << frameDiff -1 << " frames!" ;
				ROS_INFO(os.str().c_str());
				lostFrames_ ++ ;
			}

			cv::Mat yep = cv_ptr->image ;
			cv::Mat eightBit ;
			yep.convertTo(eightBit,CV_8UC1);
			IplImage *im = new IplImage(eightBit);

			if (framesRecorded_ == 0) { // start of a new acquisition
				initialTimeStamp_ = cv_ptr->header.stamp;
				initialCameraSeq_ = headerFrameNum;
			}

			frameTimeStamp_ = cv_ptr->header.stamp - initialTimeStamp_;
			bufnum_time_ = (double)frameTimeStamp_.sec*1E3 + (double)frameTimeStamp_.nsec/1E6;
			double camtime = (double)cv_ptr->header.stamp.sec*1E3 + (double)cv_ptr->header.stamp.nsec/1E6;

			NameValueMetaData * md = new NameValueMetaData ;
			// This can be eventually replaced by real cropping if we do it
			md->addData("ROIX",0);
			md->addData("ROIY",0);

			bufnum_ = headerFrameNum - initialCameraSeq_;
			md->addData("bufnum",bufnum_);
//			md->addData("bufnum_camera",cv_ptr->header.seq - initialCameraSeq_ );
			md->addData("bufnum_camera",headerFrameNum );

			md->addData("bufnum_time",bufnum_time_);
			md->addData("camtime",camtime);

			lsc.newFrame(im,md);
			//lsc.addFrameToStack()

//			std::ostringstream is;
//			is << "wfov image seq is: " << headerFrameNum << ",buffer is: " << bufnum_ ;
//			ROS_INFO(is.str().c_str());
			RecordingStatus recStatus;
			recStatus.buffer = bufnum_ ;
			recStatus.elapsed_recording = bufnum_time_;
			recStatus.lost_frames = lostFrames_ ;
			recordingStatus_pub_.publish(recStatus);

			framesRecorded_ ++ ;


		} else {
			stopRecording();
			//lsc.stopRecording();
			lsc.closeOutputFile();
			recording_ = false;
			ROS_WARN("We stopped recording on the else loop!");

			// doing some diagostics stuff

			std::ostringstream fp ;
			fp << config_.path << config_.filename << ".mmf";
			std::string ye = fp.str().c_str();
			StackReader sr(fp.str().c_str());
			std::string ya = sr.diagnostics();
			std::ostringstream os ;
			for (int f=0; f < sr.getTotalFrames()-1 ; f++ ) {

				const ImageMetaData * md = sr.getMetaData(1);
				std::map<std::string, double> gimme = md->getFieldNamesAndValues();
				md = sr.getMetaData(f+1);
				gimme = md->getFieldNamesAndValues();
				os << "bufnum: " << gimme["bufnum"] << ", bufnum_camera: " << gimme["bufnum_camera"] << ", bufnum_time: " << gimme["bufnum_time"] << std::endl ;
			}
			ROS_INFO(os.str().c_str());

			std::cout << "crap";

		}

	}


	return;
}

void ProcessCameraFrame::disconnectCamera() {

	boost::lock_guard<boost::mutex> lock(connect_mutex_);
	if (cam_wfov_sub_)
	{
		ROS_INFO("We disconnect camera and issue shutdown");
		cam_wfov_sub_.shutdown();
//		cam_sub_.shutdown();

	}
}

void ProcessCameraFrame::callback(const std_msgs::Bool str)
{
	ROS_INFO("We have been summoned!");
}

void ProcessCameraFrame::recordCb()
{
	ROS_INFO("Oh yeah!");

}

// handles (un)subscribing when clients (un)subscribe
void ProcessCameraFrame::connectCb()
{
	ROS_INFO("We do the connectCb!");

/*
	boost::lock_guard<boost::mutex> lock(connect_mutex_);
	if ((image_pub_.getNumSubscribers() == 0) && (blobs_pub_.getNumSubscribers() == 0))
	{
		cam_sub_.shutdown();
	}
	else if (!cam_sub_)
	{
		// read background image if one exists
		image_background_ = cv::imread(background_image_path_,CV_LOAD_IMAGE_GRAYSCALE);

		image_transport::TransportHints hints("raw",ros::TransportHints(),getPrivateNodeHandle());
		cam_sub_ = it_in_->subscribeCamera("image_raw",queue_size_,&ProcessCameraFrame::imageCb,this,hints);
	}
*/
}

void ProcessCameraFrame::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	NODELET_DEBUG("We enter imageCb");
	if (recording_) {
		// We need to change this for time based, not frame based
		elapsedRecordingTime_ = ros::Time().now() - recordingStartTime_ ;
		if (secondsRecorded_ != elapsedRecordingTime_.sec) {
			secondsRecorded_ = elapsedRecordingTime_.sec;
			std::ostringstream ts;
			ts << "Elapsed Time: " << elapsedRecordingTime_.sec << " sec";
			ROS_INFO(ts.str().c_str());
		}

		if ( elapsedRecordingTime_.sec <= secondsToRecord_ ) {

			cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg);
//			image_msg->header.seq;
//			info_msg->header.seq;
//			std::ostringstream seqRep;
//			seqRep << "header.seq is: " << cv_ptr->header.seq << ", info_msg->header.seq is: " << info_msg->header.seq << ", image_msg->header.seq is " << image_msg->header.seq;;
//
//			ROS_INFO(seqRep.str().c_str());

			return ;

			cv::Mat yep = cv_ptr->image ;
			int headerFrameNum = cv_ptr->header.seq;
			int frameDiff = headerFrameNum - prevFrameSeq_;
			prevFrameSeq_ = headerFrameNum ;

			if (frameDiff > 1) {
				std::ostringstream os;
				os << "Skipped " << frameDiff << " frames!" ;
				ROS_INFO(os.str().c_str());
			}

			cv::Mat eightBit ;
			yep.convertTo(eightBit,CV_8UC1);
			IplImage *im = new IplImage(eightBit);
//			IplImage* iplImage;

			// We do the metadata now
			if (framesRecorded_ == 0) { // start of a new acquisition
				initialTimeStamp_ = cv_ptr->header.stamp;
				initialCameraSeq_ = cv_ptr->header.seq;
			}

			frameTimeStamp_ = cv_ptr->header.stamp - initialTimeStamp_;
			double bufnum_time = (double)frameTimeStamp_.sec*1E3 + (double)frameTimeStamp_.nsec/1E6;
			double camtime = (double)cv_ptr->header.stamp.sec*1E3 + (double)cv_ptr->header.stamp.nsec/1E6;

			NameValueMetaData * md = new NameValueMetaData ;
			md->addData("ROIX",0);
			md->addData("ROIY",0);

//			md->addData("bufnum",headerFrameNum);
//			md->addData("bufnum",framesRecorded_);
			bufnum_ = cv_ptr->header.seq - initialCameraSeq_;
			md->addData("bufnum",bufnum_);
//			md->addData("bufnum_camera",cv_ptr->header.seq - initialCameraSeq_ );
			md->addData("bufnum_camera",headerFrameNum );

			md->addData("bufnum_time",bufnum_time);
			md->addData("camtime",camtime);

			lsc.newFrame(im,md);
			//lsc.addFrameToStack()

			std::ostringstream is;
			is << "header.seq is: " << cv_ptr->header.seq << ",buffer is: " << bufnum_ ;
			ROS_INFO(is.str().c_str());
			RecordingStatus recStatus;
			recStatus.buffer = bufnum_ ;
			recStatus.elapsed_recording = secondsRecorded_;
			recordingStatus_pub_.publish(recStatus);

			framesRecorded_ ++ ;


		} else {
			stopRecording();
			//lsc.stopRecording();
			lsc.closeOutputFile();
			recording_ = false;
			ROS_WARN("We stopped recording on the else loop!");

			// doing some diagostics stuff

			std::ostringstream fp ;
			fp << config_.path << config_.filename << ".mmf";
			std::string ye = fp.str().c_str();
			StackReader sr(fp.str().c_str());
			std::string ya = sr.diagnostics();
			std::ostringstream os ;
			for (int f=0; f < sr.getTotalFrames()-1 ; f++ ) {

				const ImageMetaData * md = sr.getMetaData(1);
				std::map<std::string, double> gimme = md->getFieldNamesAndValues();
				md = sr.getMetaData(f+1);
				gimme = md->getFieldNamesAndValues();
				os << "bufnum: " << gimme["bufnum"] << ", bufnum_camera: " << gimme["bufnum_camera"] << ", bufnum_time: " << gimme["bufnum_time"] << std::endl ;
			}
			ROS_INFO(os.str().c_str());

			std::cout << "crap";

			//		StackReader sr("/home/bruno/teststack.mmf");
			//		std::string ya = sr.diagnostics();
			//ROS_ERROR(sr.diagnostics());
			//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			//cv::waitKey(3);

		}

	}



}
// We get call back here to change parameters
void ProcessCameraFrame::configCb(Config &config,uint32_t level)
{
	ROS_INFO("We reconfigure");
	boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
	config_ = config;

}

void ProcessCameraFrame::saveBackgroundImageCb(const std_msgs::Empty::ConstPtr& msg)
{
	save_background_image_ = true;
}

void ProcessCameraFrame::startRecordingCb(const std_msgs::EmptyConstPtr msg)
{
//	bool msg = inputMsg.data;
	ROS_INFO("We start recording");
	if(recording_== false) {
		//recording_ = true ;
		startRecording() ;
	}
}

void ProcessCameraFrame::stopRecordingCb(const std_msgs::EmptyConstPtr msg)
{
//	bool msg = inputMsg.data;

	if(recording_== true) {
		//recording_ = false ;
		stopRecording() ;
	}
}

// Here we decide if we're starting or stopping or whatever :)
void ProcessCameraFrame::recordingCb(const std_msgs::Bool& inputMsg)
{
//	bool msg = inputMsg.data;
//
//	if(recording_== false && msg) {
//		recording_ = true ;
//		startRecording() ;
//	}
//
//	if(recording_== true && msg == false) {
//		recording_ = false ;
//		stopRecording() ;
//	}
//

	//	recording_ = true;
}

void ProcessCameraFrame::stopRecording() {

	ROS_INFO("We will stop recording!");

	if (recording_) {
		ROS_INFO("Issued stopRecording()!");
		lsc.stopRecording();

		lsc.closeOutputFile();
		ROS_INFO("Issued closeOutputFile()!");

		disconnectCamera();
		recording_ = false;
	}
}

}


#include "mmfWriter_nodelet.h"
#include "std_msgs/Bool.h"

namespace mmf_writer {

void ProcessCameraFrame::setParameters() {

	Config config;
	{
		boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
		config = config_;
	}

}

void ProcessCameraFrame::onInit()
{
	ROS_INFO("We initialize the ProcessCameraFrame class!");
	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();
	ros::NodeHandle nh_in (nh,"camera");
	ros::NodeHandle nh_out(nh,"blob_out");
	it_in_.reset(new image_transport::ImageTransport(nh_in));
	it_out_.reset(new image_transport::ImageTransport(nh_out));
//	ros::CallbackQueueInterface & ya = getSTCallbackQueue();
	//ros::CallbackQueue& cbq =  getMTCallbackQueue();

	// We init variables:
	recording_ = false;
	instantiated_ = true;


	// read parameters, default to 500 :)
	private_nh.param("queue_size",queue_size_,500);
	private_nh.param("frames_to_record",framesToRecord_,50);
	private_nh.param<std::string>("filename",fname_,"ros_nodelet.mmf");
	private_nh.param<std::string>("path",path_,"/home/bruno/recordings/");
	private_nh.param("keyframe_interval",keyframeInterval_,12);
	private_nh.param("seconds_to_record",secondsToRecord_,300);

	// set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(config_mutex_,private_nh));
	ReconfigureServer::CallbackType f = boost::bind(&ProcessCameraFrame::configCb,this,_1,_2);
	reconfigure_server_->setCallback(f);

	// start saving a movie
	recording_sub_ = private_nh.subscribe("recording",2,&ProcessCameraFrame::recordingCb,this);


//	ros::NodeHandle& private_nh_mt = getMTPrivateNodeHandle();
	private_nh.param<std::string>("background_image_path",background_image_path_,"background.png");
	save_background_image_ = false;



}

void ProcessCameraFrame::startRecording() {
	framesRecorded_ = 0 ;
	recording_ = true;
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
	secondsToRecord_ = config.secs_to_record;
	lsc.startRecording(999999);

	firstFrame_ = true;
	ROS_INFO("We started recording a new file!");
	recordingStartTime_ = ros::Time::now();
	secondsRecorded_ = 0 ;
}

void ProcessCameraFrame::connectCamera() {
	if (!cam_sub_)
	{
		// read background image if one exists
		image_background_ = cv::imread(background_image_path_,CV_LOAD_IMAGE_GRAYSCALE);

		image_transport::TransportHints hints("raw",ros::TransportHints(),getPrivateNodeHandle());
		cam_sub_ = it_in_->subscribeCamera("image_raw",queue_size_,&ProcessCameraFrame::imageCb,this,hints);
	}
}

void ProcessCameraFrame::disconnectCamera() {
	if (cam_sub_)
	{
		cam_sub_.shutdown();
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
}

void ProcessCameraFrame::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg)
{
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
//		if (framesRecorded_ <= framesToRecord_ ) {


			// get a cv::Mat view of the source data
			cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg,sensor_msgs::image_encodings::MONO8);


			//	cv_bridge::CvImagePtr cv_ptr;
			//	 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
			cv::Mat yep = cv_ptr->image ;
			//auto ya =  cv_ptr->header;
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
//			std::ostringstream os ;
//			os << frameTimeStamp_.sec << " sec and " << frameTimeStamp_.nsec/1E6 << " ms" ;
//			ROS_INFO(os.str().c_str());
			double bufnum_time = (double)frameTimeStamp_.sec*1E3 + (double)frameTimeStamp_.nsec/1E6;
			double camtime = (double)cv_ptr->header.stamp.sec*1E3 + (double)cv_ptr->header.stamp.nsec/1E6;
			//frameTimeStamp_ = cv_ptr->header.stamp.nsec - initialTimeStamp_;
//			os.clear();
//			os << "timestamp in ms is: " << bufnum_time ;
//			ROS_INFO(os.str().c_str());

			NameValueMetaData * md = new NameValueMetaData ;
			md->addData("ROIX",0);
			md->addData("ROIY",0);

//			md->addData("bufnum",headerFrameNum);
//			md->addData("bufnum",framesRecorded_);
			md->addData("bufnum",cv_ptr->header.seq - initialCameraSeq_);

//			md->addData("bufnum_camera",cv_ptr->header.seq - initialCameraSeq_ );
			md->addData("bufnum_camera",headerFrameNum );

			md->addData("bufnum_time",bufnum_time);
			md->addData("camtime",camtime);

			lsc.newFrame(im,md);
			//lsc.addFrameToStack()

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

	if (!instantiated_) { // We do not start recording if we have just started the call, since it can default to recording true

		boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
		config_ = config;

		if(recording_== false && config_.Recording) {
			recording_ = true ;
			startRecording() ;
		}

		if(recording_== true && config_.Recording == false) {

			stopRecording() ;
		}

	} else {
		instantiated_ = false;
	}

}

void ProcessCameraFrame::saveBackgroundImageCb(const std_msgs::Empty::ConstPtr& msg)
{
	save_background_image_ = true;
}

// Here we decide if we're starting or stopping or whatever :)
void ProcessCameraFrame::recordingCb(const std_msgs::Bool& inputMsg)
{
	bool msg = inputMsg.data;

	if(recording_== false && msg) {
		recording_ = true ;
		startRecording() ;
	}

	if(recording_== true && msg == false) {
		recording_ = false ;
		stopRecording() ;
	}


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

// register nodelet
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS( mmf_writer::ProcessCameraFrame,nodelet::Nodelet)

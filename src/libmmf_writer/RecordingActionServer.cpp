#include <mmf_writer/RecordingActionServer.h>
#include <ros/ros.h>

RecordingActionServer::RecordingActionServer(std::string name, ros::NodeHandle nh_ ) :
as_(nh_, name, false),
action_name_(name)
{

	ROS_INFO("We start the server!");

	as_.registerGoalCallback(boost::bind(&RecordingActionServer::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&RecordingActionServer::preemptCB, this));

	as_.start();

	it_in_.reset(new image_transport::ImageTransport(nh_));

	connectCamera();

}

RecordingActionServer::~RecordingActionServer(void) {}

// In this function we start recording :-)
void RecordingActionServer::goalCB()
//void RecordingActionServer::goalCB(const mmf_writer::RecordActionGoalConstPtr& goal)
{
	ROS_INFO("We set a new goal");

	goal_ = as_.acceptNewGoal()->recorder_settings;

	ROS_INFO("the path in goal is: %s", goal_.path.c_str());
	ROS_INFO("the filename in goal is: %s", goal_.fileName.c_str());

	std::ostringstream finalOutput ;
	finalOutput << "path is: " << goal_.path ;
	ROS_INFO(finalOutput.str().c_str());
	boost::filesystem::path p = boost::filesystem::path(goal_.path.c_str());
	ROS_INFO("path of p is: %s", p.c_str());

	// create if it doesn't exist
	if ( ! boost::filesystem::is_directory(p)) {
		if (!boost::filesystem::create_directories(p)) {
			ROS_INFO("Could not create directory!");
			as_.setAborted();
			return;
		}
	}

	// We issue the start of a recording
	startRecording();

	return;
}


void RecordingActionServer::startRecording() {
	framesRecorded_ = 0 ;
	lostFrames_ = 0 ;
	bufnum_time_ = 0;

	/* virtual inline void setThresholds(int threshBelowBackground, int threshAboveBackground, int smallDimMinSize, int lgDimMinSize) {
	 *
	 * threshBelowBackground: pixel values less than the background value must be this much below the background to count
	 * threshAboveBackground: pixel values greater than the background value must be this much above the background to count
	 * smallDimMinSize (s), lgDimMinSize (l): regions containing non-background pixels must be at least these dimensions (sxl or lxs)
	 */
	lsc = new ThreadedLinearStackCompressor;

	lsc->setThresholds(0, 5, 2, 3); // sticking w/ defaults for now
	// We'll hard code for now and change them later on
	//	lsc->setIntervals(180, 1);
	lsc->setIntervals(goal_.keyframe_interval, 1);
	lsc->setFrameRate(goal_.fps);

	//	lsc->setFrameRate(30);

	std::ostringstream finalOutput ;
	finalOutput << goal_.path << '/' << goal_.fileName << ".mmf";
	ROS_INFO(finalOutput.str().c_str());
	lsc->setOutputFileName(finalOutput.str().c_str());

	lsc->startRecording(999999);

	//	firstFrame_ = true;
	ROS_INFO("We started recording a new file!");
	recordingStartTime_ = ros::Time::now();
	secondsRecorded_ = 0 ;
	framesRecorded_ = 0;

	// We only enable recording once we have prepped everything else
	recording_ = true;

}


void RecordingActionServer::connectCamera() {

if (!cam_sub_) {
	image_transport::TransportHints hints("raw",ros::TransportHints(),nh_);
	cam_sub_ = it_in_->subscribeCamera("image_raw",500,&RecordingActionServer::imageCb,this,hints);
}

//	if (!cam_wfov_sub_)
//	{
//		ROS_INFO("We subscribe to the camera!");
//		cam_wfov_sub_ = nh_.subscribe("image",1000,&RecordingActionServer::imageWFOVCb,this);
//	}
}

void RecordingActionServer::processFrame(const cv_bridge::CvImageConstPtr & cv_ptr,
		int  headerFrameNum) {

	//ROS_INFO("We enter processFrame");
	elapsedRecordingTime_ = ros::Time().now() - recordingStartTime_ ;
	if (secondsRecorded_ != elapsedRecordingTime_.sec) {
		secondsRecorded_ = elapsedRecordingTime_.sec;
		std::ostringstream ts;
		ts << "Elapsed Time: " << elapsedRecordingTime_.sec << " sec";
		ROS_INFO(ts.str().c_str());
	}
	//ROS_INFO("Time is %f", bufnum_time_) ;

	if ( bufnum_time_/1000 < goal_.timeToRecord ) {
		// a line worth days of work...

		int frameDiff = headerFrameNum - prevFrameSeq_;
		prevFrameSeq_ = headerFrameNum ;

		if (framesRecorded_ == 0) { // start of a new acquisition
			initialTimeStamp_ = cv_ptr->header.stamp;
			initialCameraSeq_ = headerFrameNum;
		}

		if (frameDiff > 1 && framesRecorded_ != 0) {
			std::ostringstream os;
			os << "Skipped " << frameDiff -1 << " frames!" ;
			ROS_INFO(os.str().c_str());
			lostFrames_ = lostFrames_ + (frameDiff - 1);
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

		//			bufnum_ = headerFrameNum - initialCameraSeq_;
		md->addData("bufnum",framesRecorded_);
		//			md->addData("bufnum",bufnum_);
		//		md->addData("bufnum_camera",cv_ptr->header.seq - initialCameraSeq_ );
		md->addData("bufnum_camera",headerFrameNum );

		md->addData("bufnum_time",bufnum_time_);
		md->addData("camtime",camtime);

		lsc->newFrame(im,md);

		feedback_.feedback.recording_feedback.buffer = framesRecorded_;
		feedback_.feedback.recording_feedback.elapsed_recording = bufnum_time_/1000 ;
		feedback_.feedback.recording_feedback.lost_frames = lostFrames_ ;
		as_.publishFeedback(feedback_.feedback);

		framesRecorded_ ++ ;

	} else {
		recording_ = false;
	}
}

void RecordingActionServer::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg) {

	//ROS_INFO("We enter imageCb and will feedback");

//
//	if (as_.isPreemptRequested() || !ros::ok())
//	{
//		ROS_INFO("%s: Preempted", action_name_.c_str());
//		stopRecording();
//		as_.setAborted();
//		//	        disconnectCamera();
//	}

	cv_bridge::CvImageConstPtr cv_ptr ;

	try
	    {
	      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return;
	    }

	int headerFrameNum = info_msg->header.seq;

	if (recording_) {
		processFrame(cv_ptr, headerFrameNum);

	} else {

		if (as_.isActive()) {
			stopRecording();
			result_.result.recorder_result.buffer = framesRecorded_;
			result_.result.recorder_result.elapsed_recording = bufnum_time_/1000;
			result_.result.recorder_result.lost_frames = lostFrames_ ;
			as_.setSucceeded(result_.result);
		}
	}
}

void RecordingActionServer::imageWFOVCb(const wfov_camera_msgs::WFOVImageConstPtr& wfovImg) {
	//	ROS_INFO("We enter imageWFOVCb and will feedback");


	if (as_.isPreemptRequested() || !ros::ok())
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		recording_ = false;
		stopRecording();
		as_.setAborted();
		//	        disconnectCamera();
	}

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(wfovImg->image);

	sensor_msgs::Image image_msg = wfovImg->image ;
	sensor_msgs::CameraInfo info_msg = wfovImg->info ;

	int headerFrameNum = wfovImg->frame_counter;

	if (recording_) {

		processFrame(cv_ptr, headerFrameNum);

	} else {

		//ROS_INFO("We are in the else loop of the recording");
		if (as_.isActive()) {
			ROS_INFO("the action is active, so we issue stop recordng");
			stopRecording();
			result_.result.recorder_result.buffer = framesRecorded_;
			result_.result.recorder_result.elapsed_recording = bufnum_time_/1000;
			result_.result.recorder_result.lost_frames = lostFrames_ ;

			as_.setSucceeded(result_.result);

		}

	}

}

void writeSupplementalDataFile( std::string mmf, std::string dat) {

	ROS_INFO("writeSuppDataFile, filename to load is : %s ",mmf.c_str());
	StackReader sr(mmf.c_str());
	sr.createSupplementalDataFile(dat.c_str());
	sr.closeInputFile();
	//ROS_INFO("We are done, going to delete!");
	//delete(&sr);
	ROS_INFO("We are done!");
}

void RecordingActionServer::stopRecording() {
	// We now enable things immediately
	//disconnectCamera();
	lsc->stopRecording();
	//lsc->saveDescription();
	lsc->closeOutputFile();
	delete(lsc);
	ROS_INFO("We deleted lsc");

	std::ostringstream fp, fpDat ;
	fp << goal_.path.c_str() << '/' << goal_.fileName.c_str() << ".mmf" ;
	fpDat << goal_.path.c_str() << '/' << goal_.fileName.c_str() << ".dat" ;
//	std::thread datThread = std::thread(writeSupplementalDataFile, fp.str(),fpDat.str());
//	datThread.detach(); // we don't care what happens
	datThread = boost::thread(writeSupplementalDataFile,fp.str(),fpDat.str());
	datThread.detach();


	//	ROS_INFO("filename to load is : %s ",ye.c_str());
//	StackReader sr(fp.str().c_str());
//	sr.createSupplementalDataFile(fpDat.str().c_str());
//	sr.closeInputFile();
//	delete(&sr);
//	std::string ya = sr.diagnostics();
//	std::ostringstream os ;
//	for (int f=0; f < sr.getTotalFrames()-1 ; f++ ) {
//
//		const ImageMetaData * md = sr.getMetaData(1);
//		std::map<std::string, double> gimme = md->getFieldNamesAndValues();
//		md = sr.getMetaData(f+1);
//		gimme = md->getFieldNamesAndValues();
//		os << "bufnum: " << gimme["bufnum"] << ", bufnum_camera: " << gimme["bufnum_camera"] << ", bufnum_time: " << gimme["bufnum_time"] << std::endl ;
//	}
//	ROS_INFO(os.str().c_str());

	ROS_INFO("We exit stopRecording");

}

void RecordingActionServer::preemptCB()
{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	// set the action state to preempted
	//disconnectCamera();
//	as_.setPreempted();
//	ROS_INFO("After set Preempted");

	if (recording_) {
		recording_= false;
		stopRecording();

		result_.result.recorder_result.buffer = framesRecorded_;
		result_.result.recorder_result.elapsed_recording = bufnum_time_/1000;
		result_.result.recorder_result.lost_frames = lostFrames_ ;

		as_.setAborted(result_.result);

	}
}

void RecordingActionServer::disconnectCamera() {

	//boost::lock_guard<boost::mutex> lock(connect_mutex_);
	if (cam_wfov_sub_)
	{
		ROS_INFO("We disconnect camera and issue shutdown");
		cam_wfov_sub_.shutdown();

	}

	if (cam_sub_) {
		ROS_INFO("We disconnect the generic camera and issue shutdown");
		cam_sub_.shutdown();

	}

}

void RecordingActionServer::analysisCB(const std_msgs::Float32::ConstPtr& msg)
{
	ROS_INFO("We feedback!");
	// make sure that the action hasn't been canceled
	if (!as_.isActive())
		return;
	feedback_.feedback.recording_feedback.buffer = 10;
	as_.publishFeedback(feedback_.feedback);
	//	result_.result.recorder_result.buffer = 10;
	//	as_.setSucceeded(result,"yeah");
	//
	//	data_count_++;
	//	feedback_.sample = data_count_;
	//	feedback_.data = msg->data;
	//	//compute the std_dev and mean of the data
	//	sum_ += msg->data;
	//	feedback_.mean = sum_ / data_count_;
	//	sum_sq_ += pow(msg->data, 2);
	//	feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
	//	as_.publishFeedback(feedback_);
	//
	//	if(data_count_ > goal_)
	//	{
	//		result_.mean = feedback_.mean;
	//		result_.std_dev = feedback_.std_dev;
	//
	//		if(result_.mean < 5.0)
	//		{
	//			ROS_INFO("%s: Aborted", action_name_.c_str());
	//			//set the action state to aborted
	//			as_.setAborted(result_);
	//		}
	//		else
	//		{
	//			ROS_INFO("%s: Succeeded", action_name_.c_str());
	//			// set the action state to succeeded
	//			as_.setSucceeded(result_);
	//		}
	//	}
}

//};

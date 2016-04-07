/*
 * RecordingActionServer.h
 *
 *  Created on: Mar 24, 2016
 *      Author: bruno
 */

#ifndef MMF_WRITER_INCLUDE_MMF_WRITER_RECORDINGACTIONSERVER_H_
#define MMF_WRITER_INCLUDE_MMF_WRITER_RECORDINGACTIONSERVER_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <mmf_writer/RecordAction.h>
#include "mmf_writer/RecorderSettings.h"
#include <wfov_camera_msgs/WFOVImage.h>
#include <boost/filesystem.hpp>
#include "mmf_writer/mmfWriter.h"
#include "mmf_writer/NameValueMetaData.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>

class RecordingActionServer
{
public:
	RecordingActionServer(std::string name, ros::NodeHandle nh_ ) ;
	//	RecordingActionServer(std::string name) ;// : as_(nh_, name, false), action_name_(name) {} ;
	void connectCamera();
	void disconnectCamera();
	void imageWFOVCb(const wfov_camera_msgs::WFOVImageConstPtr& wfovImg) ;
	void imageWFOVCbPtr(const wfov_camera_msgs::WFOVImageConstPtr& wfovImg) ;

	void goalCB();
	//void goalCB(const mmf_writer::RecordActionGoalConstPtr& goal);

	void startRecording();
	void stopRecording();
	virtual ~RecordingActionServer();
	void preemptCB();
	void analysisCB(const std_msgs::Float32::ConstPtr& msg);

protected:

	ros::Subscriber cam_wfov_sub_;
	image_transport::CameraSubscriber cam_sub_;
	boost::shared_ptr<image_transport::ImageTransport> it_in_,it_out_;

	void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg);

	void processFrame(const cv_bridge::CvImageConstPtr & cv_ptr, int headerFrameNum);

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<mmf_writer::RecordAction> as_;
	std::string action_name_;
	int data_count_ , lostFrames_;
	mmf_writer::RecorderSettings goal_;
	float sum_, sum_sq_;
	mmf_writer::RecordActionFeedback feedback_;
	mmf_writer::RecordActionResult result_;
	mmf_writer::RecordingStatus * result;

	// MMF stuff
	ThreadedLinearStackCompressor * lsc;
	long framesRecorded_ , secondsRecorded_, prevFrameSeq_, initialCameraSeq_, bufnum_;
	bool recording_ , firstFrame_;
	ros::Time recordingStartTime_, initialTimeStamp_;
	ros::Duration elapsedRecordingTime_, frameTimeStamp_;
	double bufnum_time_;
	boost::thread datThread;

	ros::Subscriber sub_;
} ;





#endif /* MMF_WRITER_INCLUDE_MMF_WRITER_RECORDINGACTIONSERVER_H_ */

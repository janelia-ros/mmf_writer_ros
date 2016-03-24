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

class RecordingActionServer
{
public:
	RecordingActionServer(std::string name) ;// : as_(nh_, name, false), action_name_(name) {} ;

	void goalCB();
	virtual ~RecordingActionServer();
	 void preemptCB();
	 void analysisCB(const std_msgs::Float32::ConstPtr& msg);

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mmf_writer::RecordAction> as_;
  std::string action_name_;
  int data_count_ ;
  mmf_writer::RecorderSettings goal_;
  float sum_, sum_sq_;
  mmf_writer::RecordActionFeedback feedback_;
  mmf_writer::RecordActionResult result_;
  mmf_writer::RecordingStatus * result;

  ros::Subscriber sub_;
} ;





#endif /* MMF_WRITER_INCLUDE_MMF_WRITER_RECORDINGACTIONSERVER_H_ */

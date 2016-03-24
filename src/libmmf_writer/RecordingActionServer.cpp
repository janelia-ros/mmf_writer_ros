#include <mmf_writer/RecordingActionServer.h>


RecordingActionServer::RecordingActionServer(std::string name) :
as_(nh_, name, false),
action_name_(name)
{
	//register the goal and feeback callbacks
//	as_.registerGoalCallback(boost::bind(&RecordingActionServer::goalCB, this));
//	as_.registerPreemptCallback(boost::bind(&RecordingActionServer::preemptCB, this));

	//subscribe to the data topic of interest
	sub_ = nh_.subscribe("/random_number", 1, &RecordingActionServer::analysisCB, this);
	as_.start();
}

RecordingActionServer::~RecordingActionServer(void) {}

// In this function we start recording :-)
void RecordingActionServer::goalCB()
{
	ROS_INFO("We set a new goal");
	// reset helper variables
	data_count_ = 0;
	sum_ = 0;
	sum_sq_ = 0;
	// accept the new goal
	goal_ = as_.acceptNewGoal()->recorder_settings;
}

void RecordingActionServer::preemptCB()
{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	// set the action state to preempted
	as_.setPreempted();
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

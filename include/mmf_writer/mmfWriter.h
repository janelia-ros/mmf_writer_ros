
#ifndef MMFWRITER_H
#define	MMFWRITER_H

#include "mmf_writer/ThreadedLinearStackCompressor.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <mmf_writer/BlankMetaData.h>
#include <cv_bridge/cv_bridge.h>
#include "mmf_writer/StackReader.h"
#include "std_msgs/Bool.h"

class mmfWriter {
public:
	ThreadedLinearStackCompressor lsc;
	mmfWriter();
	int framesToRecord, framesRecorded, prevFrameSeq  ;
	bool recording;
	virtual ~mmfWriter();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg) ;
	//virtual void startRecording(int frames);
	void startRecording();

	void recordingCallback(const std_msgs::Bool str);

	void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
	  const sensor_msgs::CameraInfoConstPtr& info_msg) ;
};




#endif /* MMFWRITER_H */

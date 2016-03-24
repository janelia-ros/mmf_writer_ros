
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <mmf_writer/advertisement_checker.h>
#include <mmf_writer/StackReader.h>
#include <mmf_writer/ImageMetaData.h>

int main(int argc, char **argv)
{

	  ros::init(argc, argv, "mmf_writer");

	  // Check for common user errors
	  if (ros::names::remap("camera") != "camera")
	  {
	    ROS_WARN("Remapping 'camera' has no effect! Start mmf_writer in the "
	             "camera namespace instead.\nExample command-line usage:\n"
	             "\t$ ROS_NAMESPACE=%s rosrun mmf_writer mmf_writer",
	             ros::names::remap("camera").c_str());
	  }
	  if (ros::this_node::getNamespace() == "/")
	  {
	    ROS_WARN("Started in the global namespace! This is probably wrong. Start mmf_writer "
	             "in the camera namespace.\nExample command-line usage:\n"
	             "\t$ ROS_NAMESPACE=my_camera rosrun mmf_writer mmf_writer");
	  }

	  // Shared parameters to be propagated to nodelet private namespaces
	  ros::NodeHandle private_nh("~");
	  XmlRpc::XmlRpcValue shared_params;
	  int queue_size;
	  if (private_nh.getParam("queue_size", queue_size))
	    shared_params["queue_size"] = queue_size;

	  nodelet::Loader manager(false); // Don't bring up the manager ROS API
	  nodelet::M_string remappings(ros::names::getRemappings());
	  nodelet::V_string my_argv;

	  std::string process_image_name = ros::this_node::getName() ;
	  remappings["camera/image"] = ros::names::resolve("image");
	  remappings["camera/image_raw"] = ros::names::resolve("image_raw");
	  remappings["camera/camera_info"] = ros::names::resolve("camera_info");
	  if (shared_params.valid())
	    ros::param::set(process_image_name, shared_params);
//	  manager.load(process_image_name, "mmf_writer/process_image", remappings, my_argv);
	  manager.load(process_image_name, "mmf_writer/mmf_writer_nodelet", remappings, my_argv);

	 //  Check for only the original camera topics
	  ros::V_string topics;
	  topics.push_back(ros::names::resolve("image_raw"));
	  topics.push_back(ros::names::resolve("camera_info"));
	  mmf_writer::AdvertisementChecker check_inputs(ros::NodeHandle(), ros::this_node::getName());
	  check_inputs.start(topics, 60.0);

	  ros::spin();
	  return 0;

/*
//  mmfWriter myMMF ;
//  myMMF.lsc.setThresholds(0, 5, 2, 3);
//  // This should set keyframes to 10, but it doesn't seem to work that way?...
//  myMMF.lsc.setIntervals(120, 1);
//  myMMF.lsc.setFrameRate(30);
////  myMMF.lsc.frameRate = 30; // not sure about the whole relationship between frame rate and intervals?...
//  myMMF.lsc.setOutputFileName("/home/bruno/teststack.mmf");
//  myMMF.framesToRecord = 2000;
//  //  myMMF.startRecording(1000);

//	StackReader sr("/home/bruno/recordings/ignore_5fps_12keyframe_300frames_Recording.mmf");
//	StackReader sr("/home/bruno/teststack.mmf");
////	StackReader sr("/home/bruno/valid.mmf");
//
//	// frameAddedTimeStamp is elapsedtime in labview
//
//	std::string ya = sr.diagnostics();
//	std::ostringstream os ;
//	for (int f=0; f < 100 ; f++ ) {
//
//		const ImageMetaData * md = sr.getMetaData(1);
//		std::map<std::string, double> gimme = md->getFieldNamesAndValues();
//		md = sr.getMetaData(f+1);
//		gimme = md->getFieldNamesAndValues();
//		os << "bufnum: " << gimme["bufnum"] << ", bufnum_time: " << gimme["bufnum_time"] << std::endl ;
//	}
//	ROS_INFO(os.str().c_str());
//
//	std::cout << "crap";
//	return 0;


  ros::init(argc, argv, "mmf_writer");
  // Check for common user errors
   if (ros::names::remap("camera") != "camera")
   {
     ROS_WARN("Remapping 'camera' has no effect! Start mmf_writer in the "
              "camera namespace instead.\nExample command-line usage:\n"
              "\t$ ROS_NAMESPACE=%s rosrun mmf_writer mmf_writer",
              ros::names::remap("camera").c_str());
   }

  if (ros::this_node::getNamespace() == "/")
  {
    ROS_WARN("Started in the global namespace! This is probably wrong. Start mmf_writer "
             "in the camera namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_camera rosrun mmf_writer mmf_writer");
  }

  // Shared parameters to be propagated to nodelet private namespaces
    ros::NodeHandle private_nh("~");
    XmlRpc::XmlRpcValue shared_params;
    int queue_size;
    if (private_nh.getParam("queue_size", queue_size))
      shared_params["queue_size"] = queue_size;


  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;


  // Rectify nodelet, image_color -> image_rect_color
  // NOTE: Explicitly resolve any global remappings here, so they don't get hidden.
  remappings["camera/image_raw"] = ros::names::resolve("image_raw");
  remappings["camera/camera_info"] = ros::names::resolve("camera_info");


   * Our template

//  ROS_ERROR("We launch the process_image nodelet!");
//  std::string process_image_name = ros::this_node::getName() + "_process_image";
//  if (shared_params.valid())
//     ros::param::set(process_image_name, shared_params);
//  manager.load(process_image_name, "mmf_writer/process_image", remappings, my_argv);


   * Our mmf writer finalized :)

  ROS_ERROR("We launch the nodelet!");
  std::string mmf_process_image_name = ros::this_node::getName() + "mmfWriter_nodelet" ;
  if (shared_params.valid())
     ros::param::set(mmf_process_image_name, shared_params);
  manager.load(mmf_process_image_name, "mmf_writer/mmfWriter_nodelet", remappings, my_argv);


  // Check for only the original camera topics
  ros::V_string topics;
  topics.push_back(ros::names::resolve("image_raw"));
  topics.push_back(ros::names::resolve("camera_info"));
  mmf_writer::AdvertisementChecker check_inputs(ros::NodeHandle(), ros::this_node::getName());
  check_inputs.start(topics, 60.0);



  ros::spin();

  return 0;
*/


}

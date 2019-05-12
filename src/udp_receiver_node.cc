#include "lib/udp_client.cc"

int main(int argc, char **argv){
  ros::init(argc, argv, "udp_receiver_node");

  std::string mode;
  ros::NodeHandle nh;
  ros::param::get("~mode", mode);

  ROS_INFO("Got [%s] mode from parameter server", mode.c_str());

  udp_receiver::Input *is = 0;
  if(mode=="socket"){
    is = new udp_receiver::Input_Socket(nh);
  }
  else if(mode=="playback"){
    is = new udp_receiver::Input_Topic(nh);
  }
  else{
    ROS_ERROR("Unrecognized mode. Please enter mode argument in launch file either socket or playback");
    return -1;
  }

  ros::Rate loop_rate(10);
  while (ros::ok()){
    is->getData();
    ros::spinOnce();
  }
  delete is;
  return 0;
}

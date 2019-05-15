#include "lib/udp_client.cc"

int main(int argc, char **argv){
  ros::init(argc, argv, "udp_topic_node");

  std::string mode;
  ros::NodeHandle nh;

  udp_receiver::Input is(nh, &data_processing::processData);

  ros::spin();

  return 0;
}

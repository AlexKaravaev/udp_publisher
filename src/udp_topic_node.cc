#include "lib/udp_client.cc"

int main(int argc, char **argv){
  ros::init(argc, argv, "udp_topic_node");

  std::string mode;
  ros::NodeHandle nh;

  udp_receiver::Input_Topic is(nh);

  is.getData();

  return 0;
}
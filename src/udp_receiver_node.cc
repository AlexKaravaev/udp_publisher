#include "lib/udp_client.cc"

int main(int argc, char **argv){
  ros::init(argc, argv, "udp_receiver_node");

  ros::NodeHandle nh;

  udp_receiver::Input_Socket is(nh);



  ros::Rate loop_rate(10);
  while (ros::ok()){
    is.getData();
    ros::spinOnce();
  }

  return 0;
}

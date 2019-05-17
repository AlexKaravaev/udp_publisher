#include "lib/udp_client.cc"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "udp_sock_listener");

  ros::NodeHandle nh;

  udp_receiver::Input is(nh);

  ros::Rate loop_rate(10);
  while (ros::ok() and is.getSockData() != -1)
  {
    ros::spinOnce();
  }

  return 0;
}

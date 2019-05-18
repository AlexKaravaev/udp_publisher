#include <udp_publisher/udp_publisher.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "udp_publisher");

  ros::NodeHandle nh;

  udp_publisher::UdpPublisher up(nh);

  while (ros::ok() and up.Execute() != -1)
    ros::spinOnce();

  return 0;
}

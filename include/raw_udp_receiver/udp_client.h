#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <string>
#include <netinet/in.h>
#include <ros/ros.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include "std_msgs/Int8MultiArray.h"
#include <poll.h>

/** Base input class for socket/playback listening**/
namespace udp_receiver{

  class Input
  {
      public:
          Input(ros::NodeHandle nh);
          virtual ~Input(){};
          // virtual function to get packet from socket/topic and publish it to another topic
          virtual int getData();

      protected:
          ros::NodeHandle nh_;
          std::string device_ip_str_;
  };

  class Input_Socket: public Input
  {
      public:
          Input_Socket(ros::NodeHandle nh);
          ~Input_Socket(){};
          int getData();
      private:
          int sockfd_;
          int port_;
          in_addr devip_;
          std::string pub_topic_name_;
          ros::Publisher socket_pub_;
  };

  class Input_Topic: Input
  {
      private:
          std::string sub_topic_name_;
  };
}
#endif

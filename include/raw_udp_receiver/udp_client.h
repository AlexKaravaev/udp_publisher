#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <string>
#include <netinet/in.h>
#include <ros/ros.h>
#include <vector>
#include "std_msgs/Int8MultiArray.h"


/** Base input class for socket/playback listening**/
namespace udp_receiver{

  class Input
  {
      public:
          Input(ros::NodeHandle nh, std::string mode);
          ~Input();

          //! getData from socket
          int getData();

          /*
            dataReceived callback for udp_data
            @ param if playback mode - std_msgs::Int8MultiArray& socket_data_msg
                    if live mode - std::vector<signed char>& socket_data_msg
          */
          template<typename T> void dataReceived(const T& socket_data_msg);
      private:
          std::string mode_;
          ros::NodeHandle nh_;
          std::vector<signed char> buffer_;
          int sockfd_;
          int port_;
          in_addr devip_;
          std::string pub_topic_name_;
          ros::Publisher socket_pub_;
          std::string device_ip_str_;
          std::string sub_topic_name_;
          ros::Subscriber playback_sub_;
  };

}
#endif

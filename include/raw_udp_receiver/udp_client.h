#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <string>
#include <netinet/in.h>
#include <ros/ros.h>
#include <vector>
#include "std_msgs/Int8MultiArray.h"

// This is how the using code should look like
//
// main()
// {
//    rcv = new Input(nh, &PacketReceived);
// }
//
// void PacketReceived(void* sender, byte[] data, int dataLength)
// {
//    // I'll do my data processing here...
// }

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
            @ param if playback mode - std_msgs::Int8MultiArray& socket_data_msg {ros message from topic}
                    if live mode - std::vector<signed char>& socket_data_msg     {raw bytes from socket}
          */
          template<typename T> void dataReceived(const T& socket_data_msg);
      private:
          //! Mode either socket or playback
          std::string m_mode;

          //! Nodehandle
          ros::NodeHandle m_nh;

          //! buffer for reading data from socket
          std::vector<signed char> m_buffer;

          //! socket filedescriptor
          int m_sockfd;

          //! port number
          int m_port;

          //! device ip, default is localhost
          in_addr devip_;
          std::string m_pub_topic_name;
          ros::Publisher m_socket_pub;
          std::string m_device_ip_str;
          std::string m_sub_topic_name;
          ros::Subscriber m_playback_sub;
  };

}
#endif

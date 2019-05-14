#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include "std_msgs/Int8MultiArray.h"
#include <ros/ros.h>

#include <string>
#include <netinet/in.h>
#include <vector>
#include <cstdint>

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

namespace defaults{
  static const int buf_size = 4096;
}

/** Base input class for socket/playback listening**/
namespace udp_receiver{
  class Input
  {
      public:
          Input(ros::NodeHandle nh, void (*processDataFcn)(void*,uint8_t*,int));
          ~Input();

      private:
          //! getData from socket
          int getData();

          void udpDataReceived(const char bytes, int bytesLength);
          void udpDataReceived_cb(const std_msgs::Int8MultiArray& data_msg);

          //! Mode either live socket or playback
          bool m_playback_mode;

          //! Nodehandle
          ros::NodeHandle m_nh;

          //! buffer for reading data from socket
          uint8_t m_buffer[defaults::buf_size];

          //! socket filedescriptor
          int m_sockfd;

          //! port number
          int m_port;

          //! device ip, default is localhost
          in_addr devip_;
          std::string m_topic_name;
          ros::Publisher m_socket_pub;
          std::string m_device_ip_str;
          ros::Subscriber m_playback_sub;
  };

}
#endif

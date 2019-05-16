#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include "std_msgs/UInt8MultiArray.h"
#include <ros/ros.h>

#include <string>
#include <cstdint>
#include <vector>
#include <netinet/in.h>

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
          Input(ros::NodeHandle nh, void (*processDataFcn)(void*,std::vector<uint8_t>&,int));
          ~Input();

          //! getData from socket
          int getData();
      
      private:
          void udpDataReceived(std::vector<uint8_t>& bytes, int dataLength);    // For raw bytes
          void udpDataReceived_cb(const std_msgs::UInt8MultiArray& data_msg);    // For ros_msg

          //! Mode either live socket or playback
          bool m_playback_mode;

          //! Nodehandle
          ros::NodeHandle m_nh;

          //! buffer for reading data from socket
          std::vector<uint8_t> m_buffer = std::vector<uint8_t>(defaults::buf_size,0);

          //! socket filedescriptor
          int m_sockfd;

          //! port number
          int m_port;

          //! device ip, default is localhost
          in_addr devip_;
          std::string m_device_ip_str;
          
          //! Ros members
          std::string m_topic_name;
          ros::Publisher m_socket_pub;
          ros::Subscriber m_playback_sub;
          const char* m_node_name;
  };

}
#endif

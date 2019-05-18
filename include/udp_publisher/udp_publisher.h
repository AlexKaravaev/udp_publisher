#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include "std_msgs/UInt8MultiArray.h"
#include <ros/ros.h>

#include <string>
#include <cstdint>
#include <vector>
#include <netinet/in.h>

namespace defaults
{
  static const std::size_t buf_size = 1048576; // Setting up buffer length for 1 MB
}

namespace udp_publisher
{
  class UdpPublisher
  {
    public:
      UdpPublisher(ros::NodeHandle nh);
      ~UdpPublisher();
    
      //! Execute method (need to be called routinely). Function will block for 100ms
      int Execute();
    
    private:
      /** @brief construct ros_msg from buffer with given length and publish it to topic
      *
      * @param packet length
      **/
      void PublishPacket(int dataLength);
    
      //! Nodehandle
      ros::NodeHandle m_nh;
    
      //! buffer for reading data from socket
      std::vector<uint8_t> m_buffer = std::vector<uint8_t>(defaults::buf_size, 0);
    
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
      const char *m_node_name;
      int m_queue_size;
  };

} // namespace udp_publisher

#endif

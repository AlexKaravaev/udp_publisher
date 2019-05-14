#include <raw_udp_receiver/udp_client.h>

#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <string>
#include <netinet/in.h>
#include <ros/ros.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <iterator>

namespace udp_receiver
{

  void PacketReceived(void* sender, uint8_t* data, int dataLength)
  {
    ROS_INFO("[%s] Started processing data", m_node_name);
    for(auto i = 0; i < dataLength; i++)
    {
      std::cout << data[i];
    }
  }

  // specialization for raw_bytes from socket
  template<>
  void Input::dataReceived< std::vector<signed char>>(const std::vector<signed char>& socket_data_msg)
  {
    ROS_INFO("[%s] Started processing udp_raw_data", m_node_name);
    std_msgs::Int8MultiArray data_msg;

    /*    Iterate through bytes and process them
    *     TODO: Mock for now, later implementation will be added
    */
    for(std::vector<signed char>::const_iterator it = socket_data_msg.begin(); it != socket_data_msg.end(); it++)
      std::cout << *it;

    // Construct ros_msg and publish to topic if in the live mode
    if (m_mode == "socket"){
      data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      data_msg.layout.dim[0].size = m_buffer.size();
      data_msg.layout.dim[0].stride = 1;
      data_msg.layout.dim[0].label = "raw_data";

      data_msg.data.clear();
      data_msg.data.insert(data_msg.data.end(), m_buffer.begin(), m_buffer.end());

      m_socket_pub.publish(data_msg);
      ROS_INFO("[%s] Published %ld bytes to topic [%s]", m_node_name, m_buffer.size(), m_pub_topic_name.c_str());
      std::cout << "\n";
    }
  }

  // specialization for socket_msg
  template<>
  void Input::dataReceived<std_msgs::Int8MultiArray>(const std_msgs::Int8MultiArray& socket_data_msg){
    ROS_INFO("[%s] Got bytes from topic [%s]: ",m_node_name, m_sub_topic_name.c_str());

    // Forward vector of bytes to dataReceived function
    Input::dataReceived(socket_data_msg.data);
  }

  /** @brief constructor for socket input
  *
  * @param nodehandle
  * @param mode either "socket" or "playback"
  **/
  Input::Input(ros::NodeHandle nh, void (*processDataFcn)(void*,uint8_t*,int))
  {

    // Set up members for node parameters
    m_nh = nh;
    m_node_name = ros::this_node::getName().c_str();

    // Load the topic name to be used for publishing or subscribing
    if (!ros::param::get("~udp_topic", m_pub_topic_name))
    {
      ROS_ERROR("[%s] Failed to retrieve topic name", m_node_name);
      return;
    }

    // Load mode from parameter server.
    // Check global namespace because it can be set from another node
    if (!ros::param::get("ifPlayback", m_playback_mode) || !ros::param::get("~ifPlayback", m_playback_mode));
    {
      ROS_ERROR("[%s] Failed to retrieve mode", m_node_name);
      return;
    }

    // Choose class behavior
    if (m_playback_mode)
    {
      // Load port_number, device_ip and udp_topic parameters
      if (!ros::param::get("~port_number", m_port))
      {
        ROS_ERROR("[%s] Failed to retrieve port number", m_node_name);
        return;
      }
      if (!ros::param::get("~device_ip", m_device_ip_str))
      {
        ROS_ERROR("[%s] Failed to retrieve device ip", m_node_name);
        return;
      }

      // Convert ip string to ip address struct
      if(inet_aton(m_device_ip_str.c_str(), &devip_) != 0 )
      {
        ROS_ERROR("[%s] Failed to convert to ip_struct", m_node_name);
        return;
      }

      // Print configuration
      ROS_INFO("[%s] Retrieved port number %d", m_node_name, m_port);
      ROS_INFO("[%s] Retrieved device_ip %s", m_node_name, m_device_ip_str.c_str());
      ROS_INFO("[%s] Retrieved udp topic name %s", m_node_name, m_topic_name.c_str());
      ROS_INFO("[%s] Opening UDP socket at port %d", m_node_name, m_port);

      // Create publisher
      m_socket_pub = nh.advertise<std_msgs::Int8MultiArray>(m_topic_name, 5);

      // Connect to UDP socket
      m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
      if (m_sockfd == -1)
      {
        ROS_ERROR("[%s] Failed to open UDP socket", m_node_name);
        return;
      }

      // Prepare the socket structure
      sockaddr_in local_addr;
      memset(&local_addr, 0, sizeof(local_addr)); // Set to zeros
      local_addr.sin_family = AF_INET;            // Host byte order
      local_addr.sin_port = htons(m_port);        // Convert to network byte order
      local_addr.sin_addr.s_addr = INADDR_ANY;    // Fill local ip

      // Bind socket to ip
      if (bind(m_sockfd, (sockaddr *)&local_addr, sizeof(sockaddr)) == -1)
      {
        ROS_ERROR("[%s] Failed to bind socket to port %d", m_node_name, m_port);
        return;
      }

      // Set fd for non-blocking mode
      if (fcntl(m_sockfd, F_SETFL, O_NONBLOCK|FASYNC) < 0)
      {
        ROS_ERROR("[%s] Failed to set non-block on socket filedescriptor", m_node_name);
        return;
      }

      ROS_INFO("[%s] Socket filedescriptor set to %d", m_node_name, m_sockfd);
    }
    else
    {
      m_playback_sub = m_nh.subscribe(m_sub_topic_name, 5, &Input::dataReceived<std_msgs::Int8MultiArray>,this);
    }

    ROS_INFO("[%s] succesfully initialized", m_node_name);
  }

  // Get data from socket and forward it to topic

  int Input::getData()
  {

    m_buffer.resize(5000);
    struct pollfd fds[1];
    fds[0].fd = m_sockfd;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 100;  //100 msec polling

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    // Wait until succesfull poll from socket fd
    while(true){
      do{
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0){
          if (errno != EINTR)
          ROS_ERROR("[%s] poll() error %s", m_node_name, strerror(errno));
          return -1;
        }
        if ((fds[0].revents & POLLERR)
         || (fds[0].revents & POLLHUP)
         || (fds[0].revents & POLLNVAL))
        {
          ROS_ERROR("[%s] Device_error", m_node_name);
          return -1;
        }
      } while((fds[0].revents & POLLIN) == 0);

      // Read from socket and push_back data to vector
      ssize_t nbytes = recvfrom(m_sockfd, &m_buffer, defaults::buf_size, 0, (sockaddr*) &sender_address,&sender_address_len);

      //Check if read was succesfull
      if (nbytes < 0 )
      {
        if (errno != EWOULDBLOCK){
          ROS_ERROR("[%s] recv failed",m_node_name);
          return -1;
        }
      }
      else
      {
        // Call dataReceived callback
        m_buffer.resize(nbytes);
        dataReceived< std::vector<signed char> >(m_buffer);
        break;
      }
    }

    return 0;
  }

  Input::~Input(void)
  {
    (void) close(m_sockfd);
  }
}

#include <udp_publisher/udp_publisher.h>

#include <iostream>
#include <iterator>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>

namespace udp_publisher
{
  void UdpPublisher::PublishPacket(int dataLength)
  {
    // Construct message layout
    std_msgs::UInt8MultiArray data_msg;
    data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    data_msg.layout.dim[0].size = dataLength;
    data_msg.layout.dim[0].stride = 1;
    data_msg.layout.dim[0].label = "raw_data";

    // Insert data
    data_msg.data.clear();
    data_msg.data.insert(data_msg.data.begin(), m_buffer.begin(), m_buffer.begin() + dataLength);

    // Publish to topic
    m_socket_pub.publish(data_msg);
    ROS_INFO("[%s] Published %d bytes to topic [%s]", m_node_name, dataLength, m_topic_name.c_str());
  }
  
  UdpPublisher::UdpPublisher(ros::NodeHandle nh)
  {
    // Set up members for node parameters
    m_nh = nh;
    m_node_name = ros::this_node::getName().c_str();
  
    // Load the topic name to be used for publishing or subscribing
    if (!ros::param::get("~udp_topic", m_topic_name))
    {
      ROS_ERROR("[%s] Failed to retrieve topic name", m_node_name);
      ros::shutdown();
      return;
    }
    // Load queue size. Default is 1
    if (!ros::param::get("~queue_size", m_queue_size))
    {
      ROS_WARN("[%s] Failed to retrieve queue_size, Using default size=1", m_node_name);
      m_queue_size = 1;
    }
    // Load port_number, device_ip and udp_topic parameters
    if (!ros::param::get("~port_number", m_port))
    {
      ROS_ERROR("[%s] Failed to retrieve port number", m_node_name);
      ros::shutdown();
      return;
    }
    if (!ros::param::get("~device_ip", m_device_ip_str))
    {
      ROS_ERROR("[%s] Failed to retrieve device ip", m_node_name);
      ros::shutdown();
      return;
    }
  
    // Convert ip string to ip address struct
    if (inet_aton(m_device_ip_str.c_str(), &devip_) == 0)
    {
      ROS_ERROR("[%s] Failed to convert to ip_struct", m_node_name);
      ros::shutdown();
      return;
    }
  
    // Print configuration
    ROS_INFO("[%s] Retrieved port number %d", m_node_name, m_port);
    ROS_INFO("[%s] Retrieved device_ip %s", m_node_name, m_device_ip_str.c_str());
    ROS_INFO("[%s] Opening UDP socket at port %d", m_node_name, m_port);
  
    // Connect to UDP socket
    m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockfd == -1)
    {
      ROS_ERROR("[%s] Failed to open UDP socket", m_node_name);
      ros::shutdown();
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
      ros::shutdown();
      return;
    }
  
    // Set fd for non-blocking mode
    if (fcntl(m_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
      ROS_ERROR("[%s] Failed to set non-block on socket filedescriptor", m_node_name);
      ros::shutdown();
      return;
    }
  
    ROS_INFO("[%s] Socket filedescriptor set to %d", m_node_name, m_sockfd);
  
    // Create publisher
    m_socket_pub = nh.advertise<std_msgs::UInt8MultiArray>(m_topic_name, m_queue_size);
  
    ROS_INFO("[%s] Subscribing to {%s} topic with queue size {%d}", m_node_name, m_topic_name.c_str(), m_queue_size);
    ROS_INFO("[%s] Succesfully initialized", m_node_name);
  }
  
  int UdpPublisher::Execute()
  {
    struct pollfd fds[1];
    fds[0].fd = m_sockfd;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 100; //100 msec polling
  
    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);
  
    // Poll on socket
    int retval = poll(fds, 1, POLL_TIMEOUT);
    if (retval < 0)
    {
      // Log error (Unless terminated due to Ctrl+C)
      if (errno != EINTR)
        ROS_ERROR("[%s] poll() error %s", m_node_name, strerror(errno));

      return -1;
    }

    // Check if socket is still valid
    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))
    {
      ROS_ERROR("[%s] Device_error", m_node_name);
      return -1;
    }

    // Nothing to read? return 0
    if ((fds[0].revents & POLLIN) == 0)
      return 0;

    // Read from socket and push_back data to vector
    ssize_t nbytes = recvfrom(m_sockfd, m_buffer.data(), m_buffer.size(), 0, (sockaddr *)&sender_address, &sender_address_len);
    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        ROS_ERROR("[%s] recv failed", m_node_name);
        return -1;
      }
    }

    // Publish new packet
    PublishPacket(nbytes);
  
    return 0;
  }
  
  UdpPublisher::~UdpPublisher(void)
  {
    (void)close(m_sockfd);
    m_sockfd = -1;
  }
} // namespace udp_publisher

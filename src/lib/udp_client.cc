
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

#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

#include <raw_udp_receiver/udp_client.h>

namespace udp_receiver{

  // specialization for socket_msg
  template<>
  void Input::dataReceived<std_msgs::Int8MultiArray>(const std_msgs::Int8MultiArray& socket_data_msg){
    ROS_INFO("[%s] Got bytes from topic: ",ros::this_node::getName().c_str());
    for(std::vector<signed char>::const_iterator it = socket_data_msg.data.begin(); it != socket_data_msg.data.end(); it++){
      std::cout << *it << ' ';
    }
    std::cout << "\n";
  }

  // specialization for raw_bytes from socket
  template<>
  void Input::dataReceived<std::vector<signed char>>(const std::vector<signed char>& socket_data_msg){
    ROS_INFO("[%s] Got bytes", ros::this_node::getName().c_str());
    for(std::vector<signed char>::const_iterator it = socket_data_msg.begin(); it != socket_data_msg.end(); it++){
      std::cout << *it << ' ';
    }
    std::cout << "\n";
  }

  /** @brief constructor for socket input
  *
  * @param nodehandle
  * @param mode either "socket" or "playback"
  **/
  Input::Input(ros::NodeHandle nh, std::string mode){

    nh_ = nh;
    mode_ = mode;

    if(mode=="socket"){
      //Load port number
      if (ros::param::get("~port_number", port_)){
        ROS_INFO("[%s] Retrieved port number %d", ros::this_node::getName().c_str(), port_);
      }
      else {
        ROS_ERROR("[%s] Failed to retrieve port number", ros::this_node::getName().c_str());
        return;
      }

      //Load device ip
      if (ros::param::get("~device_ip", device_ip_str_)){
        ROS_INFO("[%s] Retrieved device_ip %s", ros::this_node::getName().c_str(), device_ip_str_.c_str());
      }
      else {
        ROS_ERROR("[%s] Failed to retrieve device ip", ros::this_node::getName().c_str());
        return;
      }

      //Load published topic name
      if (ros::param::get("~udp_topic", pub_topic_name_)){
        ROS_INFO("[%s] Retrieved udp topic name %s", ros::this_node::getName().c_str(), pub_topic_name_.c_str());
      }
      else {
        ROS_ERROR("[%s] Failed to retrieve topic name", ros::this_node::getName().c_str());
        return;
      }

      // Create publisher
      socket_pub_ = nh.advertise<std_msgs::Int8MultiArray>(pub_topic_name_, 5);

      // Connect to UDP socket
      sockfd_ = -1;

      inet_aton(device_ip_str_.c_str(), &devip_);

      ROS_INFO("[%s] Opening UDP socket at port %d", ros::this_node::getName().c_str(), port_);

      sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

      if (sockfd_ == -1){
        ROS_ERROR("[%s] Failed to open UDP socket", ros::this_node::getName().c_str());
        return;
      }

      // Local address
      sockaddr_in local_addr;
      memset(&local_addr, 0, sizeof(local_addr)); // Set to zeros
      local_addr.sin_family = AF_INET;            // Host byte order
      local_addr.sin_port = htons(port_);         // Convert to network byte order
      local_addr.sin_addr.s_addr = INADDR_ANY;    // Fill local ip

      // Bind socket to ip
      if (bind(sockfd_, (sockaddr *)&local_addr, sizeof(sockaddr)) == -1){
        ROS_ERROR("[%s] Failed to bind socket to port %d", ros::this_node::getName().c_str(), port_);
        return;
      }

      // Set fd for non-blocking mode
      if (fcntl(sockfd_, F_SETFL, O_NONBLOCK|FASYNC) < 0){
        ROS_ERROR("[%s] Failed to set non-block on socket filedescriptor", ros::this_node::getName().c_str());
        return;
      }


      ROS_INFO("[%s] Socket filedescriptor set to %d", ros::this_node::getName().c_str(), sockfd_);
    }

    else if(mode=="playback"){
      //Load subscribed topic
      if (ros::param::get("~playback_topic", sub_topic_name_)){
        ROS_INFO("[%s] Retrieved playback topic name %s", ros::this_node::getName().c_str(), sub_topic_name_.c_str());
      }
      else {
        ROS_ERROR("Failed to retrieve playback topic name");
        return;
      }
      ROS_INFO("[%s] succesfully initialized", ros::this_node::getName().c_str());
      playback_sub_ = nh_.subscribe(sub_topic_name_, 5, &Input::dataReceived<std_msgs::Int8MultiArray>,this);
    }
    ROS_INFO("[%s] succesfully initialized", ros::this_node::getName().c_str());


  }

  // Get data from socket and forward it to topic

  int Input::getData(){
    std::vector<char> buffer(5000);
    std_msgs::Int8MultiArray socket_data_msg;



    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000;

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    // Wait until succesfull poll from socket fd
    while(true){
      do{
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0){
          if (errno != EINTR)
            ROS_ERROR("[%s] poll() error %s", ros::this_node::getName().c_str(), strerror(errno));
          return -1;
        }
        if (retval == 0)
        {
          ROS_WARN("[%s] poll() timeout", ros::this_node::getName().c_str());
        }
        if ((fds[0].revents & POLLERR)
         || (fds[0].revents & POLLHUP)
         || (fds[0].revents & POLLNVAL)){
           ROS_ERROR("[%s] Device_error", ros::this_node::getName().c_str());
           return -1;
        }
      } while((fds[0].revents & POLLIN) == 0);

      // Read from socket and push_back data to vector
      ssize_t nbytes = recvfrom(sockfd_, buffer.data(), buffer.size(),0,
                                (sockaddr*) &sender_address,&sender_address_len);

      //Check if read was succesfull
      if(nbytes < 0 ){
        if (errno != EWOULDBLOCK){
          ROS_ERROR("[%s] recv failed",ros::this_node::getName().c_str());
          return -1;
        }
      }
      else{
        // Construct ros_msg and publish
        buffer.resize(nbytes);
        socket_data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        socket_data_msg.layout.dim[0].size = buffer.size();
        socket_data_msg.layout.dim[0].stride = 1;
        socket_data_msg.layout.dim[0].label = "data";

        socket_data_msg.data.clear();
        socket_data_msg.data.insert(socket_data_msg.data.end(), buffer.begin(), buffer.end());

        socket_pub_.publish(socket_data_msg);
        ROS_INFO("[%s] Published %ld bytes from socket to topic", ros::this_node::getName().c_str(), nbytes);
        break;
      }

    }

    return 0;


  }


  Input::~Input(void)
  {
    (void) close(sockfd_);
  }



}

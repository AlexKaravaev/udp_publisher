#include <raw_udp_receiver/udp_client.h>

namespace udp_receiver{
  Input::Input(ros::NodeHandle nh): nh_(nh){};

  /** @brief constructor for socket input
  *
  * @param nodehandle
  **/
  Input_Socket::Input_Socket(ros::NodeHandle nh): Input(nh){
    //Load port number
    if (nh_.getParam("port_number", port_)){
      ROS_INFO("Retrived port number [%d]", port_);
    }
    else {
      ROS_ERROR("Failed to retrieve port number");
      return;
    }

    //Load device ip
    if (nh_.getParam("device_ip", device_ip_str_)){
      ROS_INFO("Retrived device_ip [%s]", device_ip_str_.c_str());
    }
    else {
      ROS_ERROR("Failed to retrieve device ip");
      return;
    }

    //Load published topic name
    if (nh_.getParam("udp_topic", pub_topic_name_)){
      ROS_INFO("Retrived udp topic name [%s]", pub_topic_name_.c_str());
    }
    else {
      ROS_ERROR("Failed to retrieve topic name");
      return;
    }

    // Create publisher
    socket_pub_ = nh.advertise<std_msgs::Int8MultiArray>(pub_topic_name_, 5);

    // Connect to UDP socket
    sockfd_ = -1;

    inet_aton(device_ip_str_.c_str(), &devip_);

    ROS_INFO("Opening UDP socket at port [%d]", port_);

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd_ == -1){
      ROS_ERROR("Failed to open UDP socket");
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
      ROS_ERROR("Failed to bind socket to port-[%d]", port_);
      return;
    }

    // Set fd for non-blocking mode
    if (fcntl(sockfd_, F_SETFL, O_NONBLOCK|FASYNC) < 0){
      ROS_ERROR("Failed to set non-block on socket filedescriptor");
      return;
    }


    ROS_INFO("Socket filedescriptor set to [%d]", sockfd_);
  }

  // Get data from socket and forward it to topic

  int Input_Socket::getData(){
    std::vector<uint8_t> buffer(5000);
    std_msgs::Int8MultiArray data;

    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000;

    sockaddr_in sender_address;
    socklen_t sender_address_len;


  }

    //Load

}
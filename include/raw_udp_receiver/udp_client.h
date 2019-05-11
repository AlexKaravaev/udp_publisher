#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <string>
#include <netinet/in.h>
#include <ros/ros.h>

/** Base input class for socket/playback listening**/

class Input
{
    public:
        Input(ros::NodeHandle nh);
        virtual ~Input(){}
        // Get packet from socket/topic and publish it to another topic 
        virtual void getPacket();

    private:
        ros::NodeHandle nh_;
        uint16_t port_;
        std::string device_ip_;
         
};

class Input_Socket: Input
{
    private:
        int sockfd_;
        in_addr device_ip_;
};

class Input_Topic: Input
{
    private:
        std::string pub_topic_name_;
};

#endif

/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Baptiste Gil on 26/08/2014
*********************************************************************/


#include <ros/ros.h>
#include <cereal_port/CerealPort.h>

#include "mine_mapping/Mines.h"

unsigned int num_mines=0;
char send_msg[2];
cereal::CerealPort device_;
//Init arduino port
bool init(std::string* port, int baudrate)
{
    try{ device_.open(port->c_str(), baudrate); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

void msgCallback(const mine_mapping::Mines& msg){
    //If the number of mines changes
    if(msg.mine.size()>num_mines){
        num_mines=msg.mine.size();
        ROS_INFO("Play Sound!");

        device_.write(send_msg,1);
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "mines_sound_subscriber");
    ros::NodeHandle n;

    // Query for serial configuration
    std::string arduino_port;
    n.param<std::string>("arduino_port", arduino_port, "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A6008kWo-if00-port0");
    int arduino_baudrate;
    n.param("arduino_baudrate", arduino_baudrate, 9600);

    send_msg[0]='1';
    send_msg[1]='\n';


    ros::Subscriber mines_sub_ = n.subscribe("mines", 100, msgCallback);

    if(!init(&arduino_port, arduino_baudrate))
    {
        ROS_FATAL("FSR Husky Arduino - %s - Failed to initialize the arduino!", __FUNCTION__);
        ROS_BREAK();
    }


    while(ros::ok()){

        ros::spinOnce();
    }
    device_.close();
}

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
#include "std_msgs/String.h"
#include "mine_mapping/Mines.h"
#include <iostream>
#include <fstream>

using namespace std;
bool flag_end_navigation=false;

void minesCallback(const mine_mapping::Mines& msg){

    if(flag_end_navigation){

        ROS_INFO("Saving text file...");

        ofstream anti_personal_mine_file;
        anti_personal_mine_file.open ("ANTI_PERSONAL_mines.txt");
        anti_personal_mine_file << "ANTI PERSONAL mines.\n";

        ofstream anti_tank_mine_file;
        anti_tank_mine_file.open ("ANTI_TANK_mines.txt");
        anti_tank_mine_file << "ANTI TANK mines.\n";


        for(int i= 0 ; i < msg.mine.size() ; i++)
        {
            char int_to_char=(unsigned int)(msg.mine.at(i).y) + 'A';

            //Force all mines to be inside the map
            if (int_to_char>'S')
                int_to_char='S';
            else if (int_to_char<'A')
                int_to_char='A';

            if(msg.type.at(i)==mine_mapping::Mines::ANTI_PERSONAL){
                anti_personal_mine_file << int_to_char << " " << (int) (msg.mine.at(i).x + 1) << "\n";

            }else if(msg.type.at(i)==mine_mapping::Mines::ANTI_TANK)
            {
                anti_tank_mine_file << int_to_char << " " << (int) (msg.mine.at(i).x + 1) << "\n";

            }
            else
            {
                ROS_INFO("Type_error %d",(int)msg.type.at(i));
                anti_tank_mine_file << int_to_char << " " << (int) (msg.mine.at(i).x + 1) << "\n";

            }

        }

        anti_personal_mine_file.close();
        anti_tank_mine_file.close();

        ROS_INFO("Done!");
    }
}

void terminates_msg_Callback(const std_msgs::String& msg){

    if(msg.data=="done"){
        flag_end_navigation=true;
        ROS_INFO("Ready to save data.");
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "mines_to_text_subscriber");
    ros::NodeHandle n;

    ros::Subscriber mines_sub_ = n.subscribe("mines", 100, minesCallback);
    ros::Subscriber end_navigation_sub_ = n.subscribe("navigation_status", 1, terminates_msg_Callback);


    while(ros::ok()){

        ros::spinOnce();

    }
}

/***
MIT License

Copyright (c) 2022 Yihao Liu, Johns Hopkins University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
***/

#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include "ros_print_color.hpp"

class MngrVisTStickerTrackercent
{
// Manages the flag of running the data acquisition of the transform
public:
    
    MngrVisTStickerTrackercent(ros::NodeHandle& n) : n_(n){}
    bool run_flag_ = false;

private:

    ros::NodeHandle& n_;

    ros::Publisher pub_out_ = n_.advertise<std_msgs::String>(
        "/AK/msg_to_send_hi_f", 5);

    ros::Subscriber sub_run_ = n_.subscribe(
        "/AK/Kinematics/Flag_viz", 2, 
        &MngrVisTStickerTrackercent::FlagCallBack, this);
    ros::Subscriber sub_sticker_trackercent_ = n_.subscribe(
        "/AK/Kinematics/T_sticker_trackercent", 2, 
        &MngrVisTStickerTrackercent::StickerTrackercentCallBack, this);
    
    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(msg->data.compare("_start__")==0) 
        {
            run_flag_ = true;
            ROS_GREEN_STREAM("[AKTRACK INFO] Visualization started."); 
        }
        if(msg->data.compare("_end__")==0) 
        {
            run_flag_ = false;
            ROS_GREEN_STREAM("[AKTRACK INFO] Visualization Ended.");
        }
    }

    void StickerTrackercentCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (run_flag_)
        {
            std_msgs::String msg_out;
            msg_out.data = "__msg_pose_" + // convert m to mm
                std::to_string(msg->point.x * 1000.0) + "_" +
                std::to_string(msg->point.y * 1000.0);
            pub_out_.publish(msg_out);
        }
    }
};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeVisTStickerTracker");
    ros::NodeHandle nh;

    // Instantiate the flag manager
    MngrVisTStickerTrackercent mngr1(nh);

    // Go in the loop, with the flag indicating wether do the calculation or not
    ros::spin();

    return 0;
}
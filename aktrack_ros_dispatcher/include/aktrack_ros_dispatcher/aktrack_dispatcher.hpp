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

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "dispatcher_utility.hpp"

class Dispatcher
{

public:

    Dispatcher(ros::NodeHandle& n);

private:

    ros::NodeHandle& n_;

    // Dispatcher receiving query signals
    ros::Subscriber sub_ak_vis_ = n_.subscribe(
        "/AK/Vis", 2, &Dispatcher::VisualizationCallBack, this);
    ros::Subscriber sub_ak_trial_ = n_.subscribe(
        "/AK/Trial", 2, &Dispatcher::TrialsCallBack, this);

    // Dispatcher sending query response signals
    ros::Publisher pub_ak_comm_ = n_.advertise<std_msgs::String>(
        "/AK/msg_to_send", 2);
        
    // Dispatcher sending query
    ros::Publisher pub_flag_trial_ = n_.advertise<std_msgs::String>(
        "/AK/Kinematics/Flag_trial", 2);
    ros::Publisher pub_flag_viz_ = n_.advertise<std_msgs::String>(
        "/AK/Kinematics/Flag_viz", 2);

    void VisualizationCallBack(const std_msgs::String::ConstPtr& msg);

    void TrialsCallBack(const std_msgs::String::ConstPtr& msg);

};

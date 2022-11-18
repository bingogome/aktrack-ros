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

#include "dispatcher_utility.hpp"
#include "aktrack_dispatcher.hpp"

#include "ros_print_color.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

Dispatcher::Dispatcher(ros::NodeHandle& n) 
    : n_(n)
{}

void Dispatcher::VisualizationCallBack(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String msg_out;
    if(msg->data.compare("_start__")==0) 
    {
        // Poke node_viz_tr_body_tool
        msg_out.data = "_start__";
        pub_flag_viz_.publish(msg_out);
    }
    else if(msg->data.compare("_end__")==0) 
    {
        // Poke node_viz_tr_body_tool
        msg_out.data = "_end__";
        pub_flag_viz_.publish(msg_out);
    }
    
}

void Dispatcher::TrialsCallBack(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String msg_out;
    if(s.rfind("_start__", 0) == 0) 
    {
        // Poke node_viz_tr_body_tool
        msg_out.data = msg->data.substr(8);
        pub_flag_trial_.publish(msg_out);
    }
    else if(msg->data.compare("_end__")==0) 
    {
        // Poke node_viz_tr_body_tool
        msg_out.data = "_end__";
        pub_flag_trial_.publish(msg_out);
    }
}


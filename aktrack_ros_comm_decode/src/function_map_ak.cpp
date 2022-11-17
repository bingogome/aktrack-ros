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

#include <map>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

#include "function_map_ak.hpp"
#include "decode_node.hpp"

std::vector<double> SubStringTokenize2Double(std::string s, std::string del = "_")
{   
    std::vector<double> ans;
    int start = 0;
    int end = s.find(del);
    while (end != -1) {
        ans.push_back(
            std::stod(s.substr(start, end - start)));
        start = end + del.size();
        end = s.find(del, start);
    }
    ans.push_back(
        std::stod(s.substr(start, end - start)));
    return ans;
}

/**
* This maps the functions to the received cmd.
*/

CommDecoderAK::CommDecoderAK(
    ros::NodeHandle& n, 
    const std::string modulesuffix,
    FuncMap opsdict) 
    : 
    CommDecoder(n, modulesuffix, opsdict) 
{
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/AK/Vis", 2));
    pubs_.push_back(
        n_.advertise<std_msgs::String>("/AK/Trial", 2));
}

FuncMap GetFuncMapAK()
{
    FuncMap fm;

    fm["VIS_START"] = VisualizeStart;
    fm["VIS_STOP"] = VisualizeStop;
    fm["TRIAL_START"] = TrialStart;
    fm["TRIAL_STOP"] = TrialStop;

    return fm;
}

void VisualizeStart(std::string& ss, PublisherVec& pubs)
{
    // pubs[0] is the publisher /AK/Vis
    std_msgs::String msg;
    msg.data = "_start__";
    pubs[0].publish(msg);
}

void VisualizeStop(std::string& ss, PublisherVec& pubs)
{
    // pubs[0] is the publisher /AK/Vis
    std_msgs::String msg;
    msg.data = "_end__";
    pubs[0].publish(msg);
}

void TrialStart(std::string& ss, PublisherVec& pubs)
{
    // pubs[1] is the publisher /AK/Trial
    std_msgs::String msg;
    msg.data = "_start__" + ss;
    pubs[1].publish(msg);
}

void TrialStop(std::string& ss, PublisherVec& pubs)
{
    // pubs[1] is the publisher /AK/Trial
    std_msgs::String msg;
    msg.data = "_end__";
    pubs[1].publish(msg);
}
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

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "ros_print_color.hpp"

std::vector<std::string> SubString(std::string s, std::string del = "_")
{   
    std::vector<std::string> ans;
    int start = 0;
    int end = s.find(del);
    while (end != -1) {
        ans.push_back(s.substr(start, end - start));
        start = end + del.size();
        end = s.find(del, start);
    }
    ans.push_back(s.substr(start, end - start));
    return ans;
}

class MngrDataAcTStickerTrackercent
{
// Manages the flag of running the data acquisition of the transform
public:
    
    MngrDataAcTStickerTrackercent(ros::NodeHandle& n) : n_(n){}
    bool run_flag_ = false;
    std::vector<std::string> v_possible_trial_ = {
        "VPM-2-L", "VPM-2-U", "VPM-2-R", "VPM-2-D", 
        "VPM-4-L", "VPM-4-U", "VPM-4-R", "VPM-4-D", 
        "VPM-12-U", "VPM-12-R", "VPM-12-L", "VPM-12-D", 
        "VPM-6-U", "VPM-6-L", "VPM-6-D", "VPM-6-R", 
        "VPM-24-U", "VPM-24-R", "VPM-24-D", "VPM-24-L", 
        "VPM-8-D", "VPM-8-L", "VPM-8-R", "VPM-8-U", 
        "VPC-U", "VPC-D", "VPC-R", "VPC-L", 
        "VPB-hfixed", "VPB-hfree"
    };
    std::vector<geometry_msgs::PointStamped> v_data_sticker_strackercent_;
    std::string timestamp_;
    std::string subjname_;
    std::string trialname_;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_run_ = n_.subscribe(
        "/AK/Kinematics/Flag_trial", 2, 
        &MngrDataAcTStickerTrackercent::FlagCallBack, this);
    ros::Subscriber sub_sticker_trackercent_ = n_.subscribe(
        "/AK/Kinematics/T_sticker_trackercent", 2, 
        &MngrDataAcTStickerTrackercent::StickerTrackercentCallBack, this);
    
    void FlagCallBack(const std_msgs::String::ConstPtr& msg)
    {
        std::vector<std::string> v_subjtimetrial = SubString(msg->data);
        if(msg->data.compare("_end__")==0)
        {
            run_flag_ = false;
            SaveAcquiredData();
            v_data_sticker_strackercent_.clear();
            ROS_GREEN_STREAM("[AKTRACK INFO] Data recording started."); 
        }
        else if (v_subjtimetrial.size()==3)
        {
            if (std::find(v_possible_trial_.begin(), v_possible_trial_.end(), 
                v_subjtimetrial[2]) != v_possible_trial_.end())
            {
                timestamp_ = v_subjtimetrial[0];
                subjname_ = v_subjtimetrial[1];
                trialname_ = v_subjtimetrial[2];
                run_flag_ = true;
                ROS_GREEN_STREAM("[AKTRACK INFO] Data recording ended."); 
            }
        }
    }

    void StickerTrackercentCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (run_flag_)
        {
            geometry_msgs::PointStamped t_sticker_trackercent;
            t_sticker_trackercent.header = msg->header;
            t_sticker_trackercent.point = msg->point;
            v_data_sticker_strackercent_.push_back(t_sticker_trackercent);
        }
    }

    void SaveAcquiredData()
    {
        if (v_data_sticker_strackercent_.size() > 0)
        {
            double start_time = v_data_sticker_strackercent_[0].header.stamp.toSec();
            std::ofstream f;
            std::string packpath = ros::package::getPath("aktrack_ros");
            f.open(packpath + "/recordeddata/" + timestamp_ + "_" + subjname_ + "_" + trialname_ + ".csv");
            for (int i=0; i<v_data_sticker_strackercent_.size(); i++)
            {
                f << 
                    std::to_string(v_data_sticker_strackercent_[i].header.stamp.toSec()-start_time)
                    << "," << std::to_string(v_data_sticker_strackercent_[i].point.x) 
                    << "," << std::to_string(v_data_sticker_strackercent_[i].point.y) 
                    << "," << std::to_string(v_data_sticker_strackercent_[i].point.z)
                    << "\n";
            }
            f.close();
            ROS_GREEN_STREAM("[AKTRACK INFO] Recorded data saved."); 
        }
    }
};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeDataAcTStickerTracker");
    ros::NodeHandle nh;

    // Instantiate the flag manager
    MngrDataAcTStickerTrackercent mngr1(nh);

    // Go in the loop, with the flag indicating wether do the calculation or not
    ros::spin();

    return 0;
}
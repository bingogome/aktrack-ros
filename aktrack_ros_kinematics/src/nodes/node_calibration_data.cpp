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
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

typedef std::map<std::string, ros::Publisher> PubMap;

void ReadAndPublishCalibrations(std::string tr_str, PubMap pubs)
{
    std::string packpath = ros::package::getPath("aktrack_ros_kinematics");
	YAML::Node f = YAML::LoadFile(packpath + "/share/" + tr_str + ".yaml");

    geometry_msgs::Pose tr;

    tr.position.x = f["x"].as<double>();
    tr.position.y = f["y"].as<double>();
    tr.position.z = f["z"].as<double>();
    tr.orientation.x = f["rx"].as<double>();
    tr.orientation.y = f["ry"].as<double>();
    tr.orientation.z = f["rz"].as<double>();
    tr.orientation.w = f["rw"].as<double>();

    pubs[tr_str].publish(tr);

    ros::spinOnce();
}

class MngrCalibrationData
{

public:

    MngrCalibrationData(ros::NodeHandle& n, PubMap& pubs) : 
        n_(n), pubs_(pubs) {}

private:

    ros::NodeHandle& n_;
    PubMap& pubs_;
    
    ros::Subscriber sub_reinit = n_.subscribe(
        "/Kinematics/Query_ReInit", 2, &MngrCalibrationData::ReInitCallback, this);
    ros::Subscriber sub_updateoffset = n_.subscribe(
        "/Kinematics/Update_TR_cntct_offset", 2, &MngrCalibrationData::ChangeOffsetCallBack, this);
    ros::Subscriber sub_reinitoffset = n_.subscribe(
        "/Kinematics/Reinit_TR_cntct_offset", 2, &MngrCalibrationData::ReinitOffsetCallBack, this);

    void ReInitCallback(const std_msgs::String::ConstPtr& msg)
    {
        if(!msg->data.compare("_reinit__")==0) return;
        for (PubMap::iterator it = pubs_.begin(); it != pubs_.end(); it++)
        {
            ReadAndPublishCalibrations(it->first, pubs_);
        }
    }
    
    void ChangeOffsetCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
        geometry_msgs::Pose out;
        out.position = msg->position;
        out.orientation = msg->orientation;
        pubs_["cntct_offset"].publish(out);
    }

    void ReinitOffsetCallBack(const std_msgs::String::ConstPtr& msg)
    {
        if(!msg->data.compare("_reinitoffset__")==0) return;
        ReadAndPublishCalibrations("cntct_offset", pubs_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NodeCalibrationData");
    ros::NodeHandle nh;

    ros::Publisher pub_cntct_offset = nh.advertise<geometry_msgs::Pose>(
        "/AK/Kinematics/TR_cntct_offset", 1, true);
    ros::Publisher pub_tracker_tip = nh.advertise<geometry_msgs::Pose>(
        "/AK/Kinematics/TR_cntct_offset", 1, true);

    PubMap pubs;
    pubs["cntct_offset"] = pub_cntct_offset;
    pubs["tracker_tip"] = pub_tracker_tip;

    MngrCalibrationData mngr(nh, pubs);

    for (PubMap::iterator it = pubs.begin(); it != pubs.end(); it++)
    {
        ReadAndPublishCalibrations(it->first, pubs);
    }

    ros::spin();
    return 0;
}
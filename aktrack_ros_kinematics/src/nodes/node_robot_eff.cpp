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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include "transform_conversions.hpp"
#include "aktrack_ros_msgs/PoseValid.h"

class MngrTrRobotEff
{
public:
    MngrTrRobotEff(ros::NodeHandle& n) : n_(n) {}
private:
    ros::NodeHandle& n_;
    ros::Publisher pub_eff = n_.advertise<aktrack_ros_msgs::PoseValid>(
        "/Kinematics/TR_derivedeff", 1, true);
    ros::Subscriber sub = n_.subscribe(
        "/Kinematics/Query_GetTargetEff", 
        10, &MngrTrRobotEff::GetTargetEffCallback, this);
    void GetTargetEffCallback(const std_msgs::String::ConstPtr& msg)
    {
        if(!msg->data.compare("_gettargeteff__")==0) return;
        
        // Calibration data (latched)
        geometry_msgs::PoseConstPtr tr_toolref_eff = ros::topic::waitForMessage<geometry_msgs::Pose>(
            "/Kinematics/TR_toolref_eff");
        geometry_msgs::PoseConstPtr tr_tool_toolref = ros::topic::waitForMessage<geometry_msgs::Pose>(
            "/Kinematics/TR_tool_toolref");
        geometry_msgs::PoseConstPtr tr_offset_tool = ros::topic::waitForMessage<geometry_msgs::Pose>(
            "/Kinematics/TR_offset_tool");
        geometry_msgs::PoseConstPtr tr_cntct_offset = ros::topic::waitForMessage<geometry_msgs::Pose>(
            "/Kinematics/TR_cntct_offset");

        ros::Rate check_valid_rate(10);
        aktrack_ros_msgs::PoseValidConstPtr tr_body_cntct = ros::topic::waitForMessage<aktrack_ros_msgs::PoseValid>(
            "/Kinematics/TR_body_cntct");
        while(!tr_body_cntct->valid)
        {
            tr_body_cntct = ros::topic::waitForMessage<aktrack_ros_msgs::PoseValid>(
                "/Kinematics/TR_body_cntct");
            check_valid_rate.sleep();
        }
        aktrack_ros_msgs::PoseValidConstPtr tr_bodyref_body = ros::topic::waitForMessage<aktrack_ros_msgs::PoseValid>(
            "/Kinematics/TR_bodyref_body");
        while(!tr_bodyref_body->valid)
        {
            tr_bodyref_body = ros::topic::waitForMessage<aktrack_ros_msgs::PoseValid>(
                "/Kinematics/TR_bodyref_body");
            check_valid_rate.sleep();
        }
        
        tf2::Transform tr_toolref_eff_ = ConvertToTf2Transform(tr_toolref_eff);
        tf2::Transform tr_tool_toolref_ = ConvertToTf2Transform(tr_tool_toolref);
        tf2::Transform tr_offset_tool_ = ConvertToTf2Transform(tr_offset_tool);
        tf2::Transform tr_cntct_offset_ = ConvertToTf2Transform(tr_cntct_offset);
        tf2::Transform tr_body_cntct_ = ConvertToTf2Transform(tr_body_cntct);
        tf2::Transform tr_bodyref_body_ = ConvertToTf2Transform(tr_bodyref_body);

        // Old eff pose (latched)
        aktrack_ros_msgs::PoseValidConstPtr tr_robbase_effold = ros::topic::waitForMessage<aktrack_ros_msgs::PoseValid>(
            "/Kinematics/TR_robbase_effold");
        while(!tr_robbase_effold->valid)
        {
            tr_robbase_effold = ros::topic::waitForMessage<aktrack_ros_msgs::PoseValid>(
                "/Kinematics/TR_robbase_effold");
            check_valid_rate.sleep();
        }
        tf2::Transform tr_robbase_effold_ = ConvertToTf2Transform(tr_robbase_effold);

        // Sensor data (real-time)
        geometry_msgs::TransformStampedConstPtr tr_pol_bodyref = ros::topic::waitForMessage<geometry_msgs::TransformStamped>(
            "/NDI/HeadRef/local/measured_cp");
        geometry_msgs::TransformStampedConstPtr tr_pol_toolref = ros::topic::waitForMessage<geometry_msgs::TransformStamped>(
            "/NDI/CoilRef/local/measured_cp");
        tf2::Transform tr_pol_bodyref_ = ConvertToTf2Transform(tr_pol_bodyref);
        tf2::Transform tr_pol_toolref_ = ConvertToTf2Transform(tr_pol_toolref);
        
        // Calculate target EFF
        tf2::Transform derivedeff_ = 
            tr_robbase_effold_ * (tr_toolref_eff_.inverse()) * (tr_pol_toolref_.inverse()) * 
            tr_pol_bodyref_ * tr_bodyref_body_ * tr_body_cntct_ * tr_cntct_offset_ * tr_offset_tool_ * 
            tr_tool_toolref_ * tr_toolref_eff_;

        geometry_msgs::Pose derivedeff = ConvertToGeometryPose(derivedeff_);

        // Publish (latch)
        aktrack_ros_msgs::PoseValid pv;
        pv.valid = true;
        pv.pose = derivedeff;
        pub_eff.publish(pv);

        // Latch for 2 seconds and stop publish
        ros::Duration(2.0).sleep();
        pv.valid = false;
        pub_eff.publish(pv);

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NodeRobotEff");
    ros::NodeHandle nh;

    MngrTrRobotEff mngr(nh);

    ros::spin();
    return 0;
}
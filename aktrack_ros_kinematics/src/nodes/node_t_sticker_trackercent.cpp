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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

#include "transform_conversions.hpp"

class MngrTStickerTrackercent
{
public:
    
    MngrTStickerTrackercent(ros::NodeHandle& n) : n_(n){}
    bool run_flag = false;

    tf2::Transform tr_pol_panelref_;
    tf2::Transform tr_pol_trackerref_;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_tr_pol_panelref_ = n_.subscribe(
        "/NDI/PanelRef/local/measured_cp", 2, &MngrTStickerTrackercent::PolPanelRefCallBack, this);
    ros::Subscriber sub_tr_pol_trackerref_ = n_.subscribe(
        "/NDI/AtKnssRef/local/measured_cp", 2, &MngrTStickerTrackercent::PolTrackerCallBack, this);

    void PolPanelRefCallBack(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        tr_pol_panelref_ = ConvertToTf2Transform(msg);
    }

    void PolTrackerCallBack(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {
        tr_pol_trackerref_ = ConvertToTf2Transform(msg);
    }
};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeTStickerTracker");
    ros::NodeHandle nh;
    ros::Rate rate(50.0);

    // Instantiate the flag manager
    MngrTStickerTrackercent mngr1(nh);

    // Initialize the requred transforms
    geometry_msgs::PoseConstPtr tr_sticker_panelref = ros::topic::waitForMessage<geometry_msgs::Pose>(
        "/AK/Kinematics/TR_sticker_panelref");
    geometry_msgs::PoseConstPtr tr_trackerref_trackercent = ros::topic::waitForMessage<geometry_msgs::Pose>(
        "/AK/Kinematics/TR_trackerref_trackercent");

    // Initialize the tf2 intermediate variables
    tf2::Transform tr_pol_panelref_;
    tf2::Transform tr_pol_trackerref_;
    tf2::Transform tr_sticker_panelref_ = ConvertToTf2Transform(tr_sticker_panelref);
    tf2::Transform tr_trackerref_trackercent_ = ConvertToTf2Transform(tr_trackerref_trackercent);

    // Initialize the result transform
    tf2::Transform tr_sticker_trackercent_;

    // Initialize the result variable and its publisher
    geometry_msgs::Pose tr_sticker_trackercent;
    geometry_msgs::PointStamped t_sticker_trackercent;
    ros::Publisher pub_sticker_trackercent = nh.advertise<geometry_msgs::PointStamped>(
        "/AK/Kinematics/T_sticker_trackercent", 10);

    mngr1.run_flag = true;
    // Go in the loop, with the flag indicating wether do the calculation or not
    while (nh.ok())
    {
        if (mngr1.run_flag)
        {
            tr_pol_panelref_ = mngr1.tr_pol_panelref_;
            tr_pol_trackerref_ = mngr1.tr_pol_trackerref_;

            tr_sticker_trackercent_ = 
                tr_sticker_panelref_ * tr_pol_panelref_.inverse() 
                * tr_pol_trackerref_ * tr_trackerref_trackercent_;
            tr_sticker_trackercent = ConvertToGeometryPose(tr_sticker_trackercent_);
            t_sticker_trackercent.point = tr_sticker_trackercent.position;
            t_sticker_trackercent.header.stamp = ros::Time::now();
            pub_sticker_trackercent.publish(t_sticker_trackercent);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

#include "transform_conversions.hpp"

class MngrTJoystick
{
public:
    
    MngrTJoystick(ros::NodeHandle& n) : n_(n){}
    bool run_flag = false;

    geometry_msgs::Pose tr_joystick_;

private:

    ros::NodeHandle& n_;
    ros::Subscriber sub_tr_joystick_ = n_.subscribe(
        "/JOY/LeftHandle/Pose", 2, &MngrTJoystick::JoystickCallBack, this);

    void JoystickCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        tr_joystick_ = msg->pose;
    }

};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "NodeTJoystick");
    ros::NodeHandle nh;
    ros::Rate rate(50.0);

    // Instantiate the flag manager
    MngrTJoystick mngr1(nh);

    // Initialize the result variable and its publisher
    geometry_msgs::Pose tr_joystick;
    geometry_msgs::PointStamped t_joystick;
    ros::Publisher pub_joystick = nh.advertise<geometry_msgs::PointStamped>(
        "/AK/Kinematics/T_joystick", 10);

    mngr1.run_flag = true;
    // Go in the loop, with the flag indicating wether do the calculation or not
    while (nh.ok())
    {
        if (mngr1.run_flag)
        {
            tr_joystick = mngr1.tr_joystick_;

            t_joystick.point = tr_joystick.position;
            t_joystick.header.stamp = ros::Time::now();
            pub_joystick.publish(t_joystick);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
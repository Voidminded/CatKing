#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <string>
#include <signal.h>

using namespace std;

ros::Publisher pubBarcode;
ros::Publisher pubKinectTilt;
ros::Publisher pubKinectLED;
ros::Publisher pubStartPatrol;
ros::Publisher pubVisServ;
std_msgs::Bool patAllowance;
string ss;
string name;
string faceState;
bool inited = false;

void faceDataExtractor(const std_msgs::String::ConstPtr& msg)
{
    name = msg->data.c_str();
    cerr << msg->data.c_str() << endl;
}

void faceStateExtractor(const std_msgs::String::ConstPtr& msg)
{
    faceState = msg->data.c_str();
}

void speaking(const std_msgs::String::ConstPtr& msg)
{
    string toSay = "rosrun sound_play say.py \"I am at " + msg->data +"\" voice_cmu_us_clb_arctic_clunits";
    system(toSay.c_str());
}

void infoExtractor(const tf::tfMessage::ConstPtr& msg)
{
    if(!msg->transforms.empty())
    {
        cout << msg->transforms.front().child_frame_id << endl;
        ss = msg->transforms.front().child_frame_id.c_str();
        std_msgs::String str;
        if(ss == "<CMD_MoveUp>")
        {
            patAllowance.data = false;
            pubStartPatrol.publish(patAllowance);
            std_msgs::Float64 angle;
            angle.data = 30;
            pubKinectTilt.publish(angle);
            ros::Duration(3).sleep();
            name = "";
            std_msgs::UInt16 LED;
            LED.data = 1;
            pubKinectLED.publish(LED);
        }
        else if(ss == "<CMD_MoveDown>")
        {
            patAllowance.data = true;
            pubStartPatrol.publish(patAllowance);
            std_msgs::Float64 angle;
            angle.data = -15;
            pubKinectTilt.publish(angle);
            ros::Duration(3).sleep();
            name = "";
            std_msgs::UInt16 LED;
            LED.data = 2;
            pubKinectLED.publish(LED);
        }
        else if(ss == "<CMD_VisServo>")
        {
            patAllowance.data = false;
            std_msgs::UInt16 LED;
            LED.data = 3;
            pubKinectLED.publish(LED);
        }
        else if(ss == "<CMD_Recognise>")
        {
            std_msgs::UInt16 LED;
            LED.data = 6;
            pubKinectLED.publish(LED);
            cerr << "+++++name : " << name << endl;
            if( faceState == "2")
            {
                string toSay = "rosrun sound_play say.py \" Hi " + name + " What can I do for you\" voice_cmu_us_clb_arctic_clunits";
                system(toSay.c_str());
                name = "";
            }
            else if( faceState == "1")
            {
                string toSay = "rosrun sound_play say.py \"Hello human, I dont know you\" voice_cmu_us_clb_arctic_clunits";
                system(toSay.c_str());
            }
            else
            {
                string toSay = "rosrun sound_play say.py \"There is no one here\" voice_cmu_us_clb_arctic_clunits";
                system(toSay.c_str());
            }
        }
        else
        {
            std_msgs::UInt16 LED;
            LED.data = 4;
            pubKinectLED.publish(LED);
            string toSay = "rosrun sound_play say.py \"" + ss + "\" voice_cmu_us_clb_arctic_clunits";
            system(toSay.c_str());
            name = "";
        }
        str.data = ss;
        pubBarcode.publish(str);
    }
}

void mapReset(const ros::TimerEvent&)
{
    cerr << "Reseting map obstacles !!!" << endl;
    system("rosservice call /move_base/clear_costmaps");
}

void patSend(const ros::TimerEvent&)
{
    pubStartPatrol.publish(patAllowance);
}

void visualServoing(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static double xx = 0,yy = 0;
    if(yy != msg.get()->pose.position.y
            || xx != msg.get()->pose.position.x)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = msg.get()->pose.position.y * -6;
        cmd.angular.z = msg.get()->pose.position.x * 6;
        pubVisServ.publish(cmd);
        yy = msg.get()->pose.position.y;
        xx = msg.get()->pose.position.x;
    }
}

void localization(geometry_msgs::PoseWithCovarianceStampedConstPtr msg)
{
    double conv = 0;
    for(int i = 0; i < msg.get()->pose.covariance.size(); i++)
        conv += fabs(msg.get()->pose.covariance.at(i));
    cerr << "Covariance : " << conv << endl;
    if(!inited)
    {
        if(conv < 0.01)
        {
            inited = true;
            string toSay = "rosrun sound_play say.py \"Localization Finished, I am calibrated now\" voice_cmu_us_clb_arctic_clunits";
            system(toSay.c_str());
        }
    }
    else if(conv > 0.03)
    {
        inited = false;
        system("rosservice call /global_localization");
        string toSay = "rosrun sound_play say.py \"I feel something is wrong with my localization, going back to calibrating phase\" voice_cmu_us_clb_arctic_clunits";
        system(toSay.c_str());
    }
}


void mySigintHandler(int sig)
{
    patAllowance.data = false;
    cerr << "Shutting Down" << endl;
    string toSay = "rosrun sound_play say.py \"Shutting Down. Goodbye everyone.\" voice_cmu_us_clb_arctic_clunits";
    system(toSay.c_str());
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Driver_module");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    std::cerr << " inited " << std::endl;
    ros::Subscriber subFaceName = n.subscribe("/faceName", 1, faceDataExtractor);
    ros::Subscriber subFaceState = n.subscribe("/faceDetected", 1, faceStateExtractor);
    ros::Subscriber subVIsServ = n.subscribe("/object_position", 1, visualServoing);
    ros::Subscriber subSpeaker = n.subscribe("sayThis", 1, speaking);
    ros::Subscriber subLocalizer = n.subscribe("amcl_pose", 1, localization);
    ros::Subscriber subBarcode = n.subscribe("/barcode", 5, infoExtractor);
    ros::Timer mapResetTimer = n.createTimer(ros::Duration(12), mapReset);
    ros::Timer patrolTimer = n.createTimer(ros::Duration(1), patSend);
    system("rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 \"none\"");
    system("rosservice call /global_localization");
    pubBarcode = n.advertise<std_msgs::String>("barcodeInfo", 100);
    pubKinectTilt = n.advertise<std_msgs::Float64>("tilt_angle",3);
    pubKinectLED = n.advertise<std_msgs::UInt16>("led_option",3);
    pubStartPatrol = n.advertise<std_msgs::Bool>("startPatrol",9);
    pubVisServ = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",6);
    patAllowance.data = true;
    std::cerr << " blah " << std::endl;
    //    tf::TransformListener tf(ros::Duration(10));
    //    costmap_2d::Costmap2DROS costmap("/move_base/local_costmap/costmap", tf);
    ros::spin();

    return 0;
}

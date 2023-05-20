/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <vector>

using namespace std;

class OrbSlamMono
{
  public:
    OrbSlamMono(){
      // Get handles
      ros::NodeHandle nhtopics("");
      ros::NodeHandle nhparams("~");

      // Get params
      nhparams.param<int>("rate", rate, 100);
      nhparams.param<std::string>("vocab_filepath", vocab_path); //, "Vocabulary/ORBvoc.txt"
      nhparams.param<std::string>("settings_filepath", settings_path); //, "Monocular/Screen.yaml"
      nhparams.param<std::string>("settings_filepath", settings_path); //, "Monocular/Screen.yaml"
      nhparams.param<std::string>("img_topic", listener_topic, "/cmd");
      nhparams.param<std::string>("pose_topic", talker_topic, "/cmd");
      nhparams.param<bool>("save_trajectory", save_trajectory, false);
      loginfo("--ORB SLAM Monocular node starting--");
      loginfo(listener_topic.c_str());

      ORB_SLAM2::System m(vocab_path,settings_path,ORB_SLAM2::System::MONOCULAR,true);
      SLAM = &m;

      // Setup subscriber
      sub = nhtopics.subscribe(listener_topic, 1, &OrbSlamMono::callback, this);
      SLAM->Shutdown();
      if (save_trajectory) {
        SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
      }

      // Setup publisher
      pub = nhtopics.advertise<geometry_msgs::PoseStamped>("orb_pose", rate);
    }

  private:
    std::string node_name = ros::this_node::getName();
    int rate;
    std::string listener_topic, talker_topic;
    bool save_trajectory;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string vocab_path, settings_path;
    float scale_factor=1.0;
    ORB_SLAM2::System* SLAM;

    void callback(const sensor_msgs::ImageConstPtr& msg) {
      cv_bridge::CvImageConstPtr cv_ptr;

      try {
          cv_ptr = cv_bridge::toCvShare(msg);
      } catch (cv_bridge::Exception& e) {
        loginfo("cv_bridge exception!");
        ROS_ERROR("Error: \n%s", e.what());
        return;
      }

      cv::Mat inferred_pose, rot_e, trans;
      inferred_pose = SLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

      rot_e = inferred_pose.rowRange(0,3).colRange(0,3).t();
      trans = -rot_e*inferred_pose.rowRange(0,3).col(3);
      vector<float> rot_q = ORB_SLAM2::Converter::toQuaternion(rot_e);

      tf::Transform transform;
      transform.setOrigin(tf::Vector3(trans.at<float>(0, 0)*scale_factor, trans.at<float>(0, 1)*scale_factor, trans.at<float>(0, 2)*scale_factor));
      tf::Quaternion rot_quaternion(rot_q[0], rot_q[1], rot_q[2], rot_q[3]);
      transform.setRotation(rot_quaternion);

      static tf::TransformBroadcaster br_;
      br_.sendTransform(tf::StampedTransform(transform, ros::Time(cv_ptr->header.stamp.toSec()), "world", "ORB_SLAM2_Mono"));

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = cv_ptr->header.stamp;
      pose.header.frame_id ="ORB_SLAM2_MONO";
      tf::poseTFToMsg(transform, pose.pose);
      pub.publish(pose);
    }

    void loginfo(const char* msg){
      ROS_INFO("[%s] %s ", node_name.c_str(), msg);
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "orb_slam_mono_node");
  OrbSlamMono mono_node;
  ros::spin();
  return 0;
}
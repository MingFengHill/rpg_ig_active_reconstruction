/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich, Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * ig_active_reconstruction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * Please refer to the GNU Lesser General Public License for details on the license,
 * on <http://www.gnu.org/licenses/>.
*/

#include <stdexcept>
#include <std_msgs/Int32.h>
#include <iostream>

#include "ig_active_reconstruction/robot_ros_client_rl.hpp"


namespace ig_active_reconstruction
{
  
namespace robot
{
  RosClientRL::RosClientRL(ros::NodeHandle nh_sub)
  : nh_sub_(nh_sub)
  {
    cur_pos_sub_ = nh_sub_.subscribe("camera_id", 1, &RosClientRL::cameraPositionUpdateCallback, this);
    next_best_view_pub_ = nh_sub_.advertise<std_msgs::Int32>("next_best_view", 1000);
  }

  void RosClientRL::cameraPositionUpdateCallback(const std_msgs::Int32::ConstPtr& camaraPosition)
  {
    std::unique_lock<std::mutex> lockGuard(camara_position_mtx_);
    cur_camera_position_ = camaraPosition->data;
    // std::cout << "cur_camera_position_" << cur_camera_position_ << std::endl;
  }
  
  int RosClientRL::getCurrentView()
  {
    int cur_position;
    {
      std::unique_lock<std::mutex> lockGuard(camara_position_mtx_);
      cur_position = cur_camera_position_;
    }
    return cur_position;
  }
  
  bool RosClientRL::moveTo(int nextBestView)
  {
    std_msgs::Int32 msg;
    msg.data = nextBestView;
    while (true) {
      next_best_view_pub_.publish(msg);
      int cur_position;
      {
        std::unique_lock<std::mutex> lockGuard(camara_position_mtx_);
        cur_position = cur_camera_position_;
      }
      if (cur_position == nextBestView) {
        break;
      }
      std::cout << "cur_position: " << cur_position << " nbv: " << nextBestView << std::endl;
      ros::Duration(1).sleep();
    };
    ros::Duration(10).sleep();
    return true;
  }
}

}
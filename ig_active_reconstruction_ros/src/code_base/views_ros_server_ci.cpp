/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of ig_active_reconstruction, a ROS package for...well,

ig_active_reconstruction is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
ig_active_reconstruction is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with ig_active_reconstruction. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdexcept>

#include "ig_active_reconstruction_ros/views_ros_server_ci.hpp"
#include "ig_active_reconstruction_ros/views_conversions.hpp"


namespace ig_active_reconstruction
{
  
namespace views
{
  
  RosServerCI::RosServerCI( ros::NodeHandle nh, boost::shared_ptr<CommunicationInterface> linked_interface )
  : nh_(nh)
  , linked_interface_(linked_interface)
  {
    viewspace_service_ = nh.advertiseService("views/space", &RosServerCI::viewspaceService, this );
    views_adder_service_ = nh.advertiseService("views/add", &RosServerCI::viewsAdderService, this );
    views_deleter_service_ = nh.advertiseService("views/delete", &RosServerCI::viewsDeleterService, this );
  }
    
  RosServerCI::ViewSpaceStatus RosServerCI::getPlanningSpace( ViewSpace* space )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("views::RosServerCI::Interface not linked.");
    
    return linked_interface_->getPlanningSpace(space);
  }
  
  void RosServerCI::getViewSpacePtr(ViewSpace* viewspace, ViewSpaceStatus& status)
  {
    if( linked_interface_ == nullptr )
    {
      status = views::CommunicationInterface::ViewSpaceStatus::NONE_AVAILABLE;
      return;
    }
    
    return linked_interface_->getViewSpacePtr(viewspace, status);
  }
  
  RosServerCI::ViewSpaceUpdateResult RosServerCI::addViews( std::vector<View>& new_views )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("views::RosServerCI::Interface not linked.");
    
    return linked_interface_->addViews(new_views);
  }
  
  RosServerCI::ViewSpaceUpdateResult RosServerCI::addView( View new_view )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("views::RosServerCI::Interface not linked.");
    
    return linked_interface_->addView(new_view);
  }
  
  RosServerCI::ViewSpaceUpdateResult RosServerCI::deleteViews( std::vector<View::IdType>& view_ids )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("views::RosServerCI::Interface not linked.");
    
    return linked_interface_->deleteViews(view_ids);
  }
  
  RosServerCI::ViewSpaceUpdateResult RosServerCI::deleteView( View::IdType view_id )
  {
    if( linked_interface_ == nullptr )
      throw std::runtime_error("views::RosServerCI::Interface not linked.");
    
    return linked_interface_->deleteView(view_id);
  }
  
  bool RosServerCI::viewspaceService( ig_active_reconstruction_msgs::ViewSpaceRequest::Request& req, ig_active_reconstruction_msgs::ViewSpaceRequest::Response& res )
  {
    if( linked_interface_ == nullptr )
    {
      ViewSpaceStatus status = ViewSpaceStatus::BAD;
      res.viewspace_status = ros_conversions::viewSpaceStatusToMsg(status);
      return true;
    }
    
    ViewSpace* viewspace;
    ViewSpaceStatus status;
    linked_interface_->getViewSpacePtr(viewspace,status);
    
    res.viewspace_status = ros_conversions::viewSpaceStatusToMsg(status);
    
    return true;
  }
  
  bool RosServerCI::viewsAdderService( ig_active_reconstruction_msgs::ViewSpaceUpdate::Request& req, ig_active_reconstruction_msgs::ViewSpaceUpdate::Response& res )
  {
    if( linked_interface_ == nullptr )
    {
      ViewSpaceUpdateResult result = ViewSpaceUpdateResult::NOT_AVAILABLE;
      res.update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
      return true;
    }
    
    std::vector<View> new_views;
    for(ig_active_reconstruction_msgs::ViewMsg& view_msg: req.views)
    {
      new_views.push_back( ros_conversions::viewFromMsg(view_msg) );
    }
    ViewSpaceUpdateResult result = linked_interface_->addViews(new_views);
    
    res.update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
    return true;
  }
  
  bool RosServerCI::viewsDeleterService( ig_active_reconstruction_msgs::DeleteViews::Request& req, ig_active_reconstruction_msgs::DeleteViews::Response& res )
  {
    if( linked_interface_ == nullptr )
    {
      ViewSpaceUpdateResult result = ViewSpaceUpdateResult::NOT_AVAILABLE;
      res.update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
      return true;
    }
    
    std::vector<View::IdType> delete_ids;
    for(uint64_t& id:req.ids)
    {
      delete_ids.push_back(id);
    }
    ViewSpaceUpdateResult result = linked_interface_->deleteViews(delete_ids);
    res.update_result = ros_conversions::viewSpaceUpdateResultToMsg(result);
    
    return true;
  }
  
}

}
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

#include "ig_active_reconstruction/benchmark_view_planner.hpp"

#include <chrono>
#include <boost/smart_ptr.hpp>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/chrono/include.hpp>

namespace ig_active_reconstruction
{
  
  BenchmarkViewPlanner::Config::Config()
  : discard_visited(false)
  , max_visits(-1)
  {
  }
  
  BenchmarkViewPlanner::BenchmarkViewPlanner( ros::NodeHandle nh, Config config )
  : config_(config)
  , robot_comm_unit_(nullptr)
  , views_comm_unit_(nullptr)
  , world_comm_unit_(nullptr)
  , utility_calculator_(nullptr)
  , goal_evaluation_module_(nullptr)
  , status_(Status::UNINITIALIZED)
  , runProcedure_(false)
  , pauseProcedure_(false)
  , nh_(nh)
  , view_update_(false)
  {
    views_space_sub_ = nh_.subscribe("views/clear", 1, &BenchmarkViewPlanner::viewsSpaceClearCallback, this);
  }

  void BenchmarkViewPlanner::viewsSpaceClearCallback(const std_msgs::Int32::ConstPtr& clear_token)
  {
    std::unique_lock<std::mutex> lockGuard(view_update_mutex_);
    view_update_ = true;
    std::cout << "BenchmarkViewPlanner::viewsSpaceClearCallback" << std::endl;
  }

  
  BenchmarkViewPlanner::~BenchmarkViewPlanner()
  {
    runProcedure_ = false;
    if( running_procedure_.joinable() )
      running_procedure_.join();
  }
  
  void BenchmarkViewPlanner::setRobotCommUnit( boost::shared_ptr<robot::RosClientRL> robot_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    robot_comm_unit_ = robot_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BenchmarkViewPlanner::setViewsCommUnit( boost::shared_ptr<views::CommunicationInterface> views_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    views_comm_unit_ = views_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BenchmarkViewPlanner::setWorldCommUnit( boost::shared_ptr<world_representation::CommunicationInterface> world_comm_unit )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    world_comm_unit_ = world_comm_unit;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BenchmarkViewPlanner::setUtility( boost::shared_ptr<UtilityCalculator> utility_calculator )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    utility_calculator_ = utility_calculator;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  void BenchmarkViewPlanner::setGoalEvaluationModule( boost::shared_ptr<GoalEvaluationModule> goal_evaluation_module )
  {
    if( runProcedure_ || running_procedure_.joinable() )
      return;
    
    goal_evaluation_module_ = goal_evaluation_module;
    
    if( isReady() && status_==Status::UNINITIALIZED )
      status_ = Status::IDLE;
    else
      status_ = Status::UNINITIALIZED;
  }
  
  bool BenchmarkViewPlanner::run()
  {
    if( runProcedure_ || running_procedure_.joinable() )
    {
      if( pauseProcedure_ )
      {
	pauseProcedure_ = false;
	return true;
      }
      
      return false;
    }
    
    runProcedure_ = true;
    running_procedure_ = std::thread(&BenchmarkViewPlanner::main, this);
    
    return true;
  }
  
  void BenchmarkViewPlanner::pause()
  {
    pauseProcedure_ = true;
  }
  
  void BenchmarkViewPlanner::stop()
  {
    runProcedure_ = false;
  }
  
  BenchmarkViewPlanner::Status BenchmarkViewPlanner::status()
  {
    return status_;
  }
  
  bool BenchmarkViewPlanner::isReady()
  {
    return robot_comm_unit_!=nullptr
	&& views_comm_unit_!=nullptr
	&& world_comm_unit_!=nullptr
	&& utility_calculator_!=nullptr
	&& goal_evaluation_module_!=nullptr;
  }
  
  void BenchmarkViewPlanner::main()
  {    
    // preparation
    goal_evaluation_module_->reset();
    
    // get viewspace................................................
    viewspace_ = boost::make_shared<views::ViewSpace>();
    views::CommunicationInterface::ViewSpaceStatus viewspace_status;
    do
    {
      status_ = Status::DEMANDING_VIEWSPACE;
      *viewspace_ = views_comm_unit_->getViewSpace();
      
      if( !runProcedure_ ) // exit point
	{
	  status_ = Status::IDLE;
	  runProcedure_ = false;
	  return;
	}
      pausePoint();
      
    }while( viewspace_->empty() );

//     std::cout << "viewspace: " << viewspace_->size() << std::endl;
//     int first_id = robot_comm_unit_->getCurrentView();
//     std::cout << "first step: " << first_id << std::endl;
//     viewspace_->setBad(first_id * 2);
    int first_id = 0;

    
    unsigned int reception_nr = 0;

    int step_num = 10;
    int cur_step = 10;
    
    do
    {
      if (cur_step == step_num)
      {
        cur_step = 1;
        viewspace_->clear();
        do {
          int is_update = false;
          {
            std::unique_lock<std::mutex> lockGuard(view_update_mutex_);
            is_update = view_update_;
          }
          if (is_update) {
            std::unique_lock<std::mutex> lockGuard(view_update_mutex_);
            view_update_ = false;
            first_id = robot_comm_unit_->getCurrentView();
            std::cout << "first step: " << first_id << std::endl;
            viewspace_->setBad(first_id * 2);
            break;
          }
        } while (true);
        {
          std::unique_lock<std::mutex> lockGuard(view_update_mutex_);
          view_update_ = false;
        }
      }
      cur_step++;
      // determine view candidate subset of viewspace .....................
      views::ViewSpace::IdSet view_candidate_ids;
      viewspace_->getGoodViewSpace(view_candidate_ids, config_.discard_visited);
      
      if(view_candidate_ids.empty())
      {
	break;
      }
      
      std::cout<<"\nData reception nr. "<<++reception_nr<<".";
      
      // getting cost and ig is wrapped in the utility calculator..................
      status_ = Status::NBV_CALCULATIONS;
      std::cout << "candidate id: " << std::endl;
      for (auto& view_id : view_candidate_ids)
      {
        std::cout << view_id << " ";
      }
      std::cout << std::endl;
      std::cout << "--------------------" << std::endl;

      views::View::IdType nbv_id = utility_calculator_->getNbv(view_candidate_ids,viewspace_);
      views::View nbv = viewspace_->getView(nbv_id);
      
      // check termination criteria ...............................................
//       if( goal_evaluation_module_->isDone() )
//       {
// 	std::cout<<"\n\nTermination criteria was fulfilled. Reconstruction procedure ends.\n\n";
// 	break;
//       }
      
      // move to next best view....................................................
      bool successfully_moved = false;
      do
      {
	status_ = Status::DEMANDING_MOVE;
	successfully_moved = robot_comm_unit_->moveTo(nbv_id/2);
	
	if( !runProcedure_ ) // exit point
	{
	  status_ = Status::IDLE;
	  runProcedure_ = false;
	  return;
	}
	pausePoint();
	
      }while(!successfully_moved);
      
      // update viewspace
      viewspace_->setVisited(nbv_id);
      if( config_.max_visits!=-1 && viewspace_->timesVisited(nbv_id) >= config_.max_visits )
	viewspace_->setBad(nbv_id);
      
    }while( runProcedure_ );
    
    status_ = Status::IDLE;
    runProcedure_ = false;
    return;
  }
  
  void BenchmarkViewPlanner::pausePoint()
  {
    while(pauseProcedure_)
    {
      status_ = Status::PAUSED;
      boost::this_thread::sleep_for( boost::chrono::seconds(1) );
    }
  }
}
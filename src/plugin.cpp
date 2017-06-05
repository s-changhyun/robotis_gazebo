/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <urdf/model.h>
#include <string.h>

#include <robotis_controller/robotis_controller.h>
#include <robotis_gazebo/joint_handle.h>

/* Sensor Module Header */

/* Motion Module Header */
#include "robotis_op3_motion_module/motion_module.h"

namespace gazebo
{

class RobotisGazeboPlugin : public ModelPlugin
{
public:
  RobotisGazeboPlugin();
  ~RobotisGazeboPlugin();
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  virtual void Init();

private:
  void OnUpdate(const common::UpdateInfo& info);

  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  common::Time prevUpdateTime;

  int cnt_;

  std::vector<JointHandlePtr> joints_;
  //  robotis_framework::RobotisController controller_manager_;
  robotis_framework::RobotisController *controller_ = robotis_framework::RobotisController::getInstance();
  ros::Time last_update_time_;

  ros::Publisher joint_state_pub_;

  ros::NodeHandle nh_;
  ros::Time last_publish_;

  boost::mutex queue_mutex_;
};

RobotisGazeboPlugin::RobotisGazeboPlugin()
{
}

RobotisGazeboPlugin::~RobotisGazeboPlugin()
{
}

void RobotisGazeboPlugin::Load(
    physics::ModelPtr parent,
    sdf::ElementPtr sdf)
{
  // Need to hang onto model
  model_ = parent;

  // Update each simulation iteration
  update_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RobotisGazeboPlugin::OnUpdate, this, _1));
}

void RobotisGazeboPlugin::Init()
{
  cnt_ = 0;

  // Init time stuff
  prevUpdateTime = model_->GetWorld()->GetSimTime();
  last_publish_ = ros::Time(prevUpdateTime.Double());
  urdf::Model urdfmodel;
  if (!urdfmodel.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
  }

  // Init joint handles
  gazebo::physics::Joint_V joints = model_->GetJoints();
  for (physics::Joint_V::iterator it = joints.begin(); it != joints.end(); ++it)
  {
    //get effort limit and continuous state from URDF
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdfmodel.getJoint((*it)->GetName());

    if (urdf_joint->type != urdf::Joint::FIXED)
    {
      JointHandlePtr handle(new JointHandle(*it,
                                            urdf_joint->limits->velocity,
                                            urdf_joint->limits->effort,
                                            (urdf_joint->type == urdf::Joint::CONTINUOUS)));

      joints_.push_back(handle);
      robotis_framework::JointHandlePtr h(handle);
      controller_->addJointHandle(h);
    }
  }

  // Init controllers
  /* Load ROS Parameter */
  std::string offset_file = nh_.param<std::string>("offset_file_path", "");
  std::string robot_file  = nh_.param<std::string>("robot_file_path", "");

  std::string init_file   = nh_.param<std::string>("init_file_path", "");

  /* gazebo simulation */
  controller_->gazebo_mode_ = nh_.param<bool>("is_gazebo", false);
  if(controller_->gazebo_mode_ == true)
  {
    ROS_WARN("SET TO GAZEBO MODE!");
    std::string robot_name = nh_.param<std::string>("gazebo_robot_name", "");
    if(robot_name != "")
      controller_->gazebo_robot_name_ = robot_name;
  }

  if(robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return;
  }

  if(controller_->initialize(robot_file, init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return;
  }

  if(offset_file != "")
    controller_->loadOffset(offset_file);

  sleep(1);

  /* Add Sensor Module */

  /* Add Motion Module */
  controller_->addMotionModule((robotis_framework::MotionModule*)robotis_op3::MotionModule::getInstance());
  controller_->startTimer();

  // Publish joint states only after controllers are fully ready
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

  ROS_INFO("Finished initializing RobotisGazeboPlugin");
}

void RobotisGazeboPlugin::OnUpdate(const common::UpdateInfo& info)
{
  if (!ros::ok())
    return;

  // Get time and timestep for controllers
  common::Time currTime = model_->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - prevUpdateTime;
  prevUpdateTime = currTime;
  double dt = stepTime.Double();
  ros::Time now = ros::Time(currTime.Double());

  //  ROS_INFO("---");

  queue_mutex_.lock();

  //  ros::Time begin = ros::Time::now();
  //  ros::Duration time_duration;

  //  if (cnt_ < 10)
  //  {
  //    ROS_INFO("========== cnt : %d ==========", cnt_);

  // Update controllers
  controller_->update(now, ros::Duration(dt));

  // Update joints back into Gazebo
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    //      ROS_INFO("joints_size : %d", (int) joints_.size());

    //      ROS_INFO("[Plugin] joint name : %s", (joints_[i]->getName()).c_str());
    joints_[i]->update(now, ros::Duration(dt));
  }

  //  time_duration = ros::Time::now() - begin;
  //  ROS_INFO("time : %f", time_duration.sec + time_duration.nsec * 1e-9);
  //  }

  queue_mutex_.unlock();

  //  cnt_++;

  // Limit publish rate
  //  if (now - last_publish_ < ros::Duration(0.01))
  //    return;

  //  // Publish joint_state message
  //  sensor_msgs::JointState js;
  //  js.header.stamp = ros::Time(currTime.Double());
  //  for (size_t i = 0; i < joints_.size(); ++i)
  //  {
  //    js.name.push_back(joints_[i]->getName());
  //    js.position.push_back(joints_[i]->getPosition());
  //    js.velocity.push_back(joints_[i]->getVelocity());
  //    js.effort.push_back(joints_[i]->getEffort());
  //  }
  //  joint_state_pub_.publish(js);

  //  last_publish_ = now;
}

GZ_REGISTER_MODEL_PLUGIN(RobotisGazeboPlugin)

}  // namespace gazebo

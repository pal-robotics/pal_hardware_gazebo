/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
/** \author Sai Kishor Kothakota **/

#include <cassert>

#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

#include <joint_limits_interface/joint_limits_urdf.h>

#include <pal_hardware_gazebo/pal_hardware_transmission_gazebo.h>

namespace gazebo_ros_control
{
int getJointIndex(const std::vector<std::string> &joint_names, const std::string &joint_name)
{
  auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
  if (it == joint_names.end())
  {
    // If the joint is not found in the parsed names
    return -1;
  }
  else
  {
    return std::distance(joint_names.begin(), it);
  }
}

using namespace hardware_interface;

PalHardwareTransmissionGazebo::PalHardwareTransmissionGazebo()
  : gazebo_ros_control::RobotHWSim()
{
}

bool PalHardwareTransmissionGazebo::initSim(
    const std::string &robot_ns, ros::NodeHandle nh, gazebo::physics::ModelPtr model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  using gazebo::physics::JointPtr;

  // Wait for robot model to become available
  const std::string robot_description_name = "robot_description";
  std::string robot_description;
  while (ros::ok() && !nh.getParam(robot_description_name, robot_description))
  {
    ROS_WARN_STREAM_ONCE("Waiting for robot description: parameter '"
                         << robot_description_name << "' on namespace '"
                         << nh.getNamespace() << "'.");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Found robot description");

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(robot_description);
  if (!urdf)
  {
    throw std::runtime_error("Could not load robot description.");
  }
  ROS_INFO("Parsed robot description");

  // Cleanup
  sim_joints_.clear();
  jnt_types_.clear();
  jnt_pos_.clear();
  jnt_vel_.clear();
  jnt_eff_.clear();
  act_pos_.clear();
  act_vel_.clear();
  act_eff_.clear();
  jnt_pos_cmd_.clear();
  jnt_eff_cmd_.clear();
  joint_names_.clear();
  jnt_ctrl_mthd_.clear();
  transmissions_data_.clear();
  transmissions_data_.resize(transmissions.size());

  // Check the hardware interface is of proper type in the transmission
  for (const transmission_interface::TransmissionInfo &transmission : transmissions)
  {
    for (size_t i = 0; i < transmission.joints_.size(); i++)
    {
      if (transmission.joints_[i].hardware_interfaces_.empty())
      {
        ROS_ERROR_STREAM("No hardware interfaces found for the joint : "
                         << transmission.joints_[i].name_);
        return false;
      }
      bool interface_status = false;
      for (const std::string &hw_inf : transmission.joints_[i].hardware_interfaces_)
      {
        if ((hw_inf == "hardware_interface/PositionJointInterface") ||
            (hw_inf == "hardware_interface/EffortJointInterface"))
        {
          interface_status = true;
          break;
        }
      }
      if (!interface_status)
      {
        ROS_WARN_STREAM("The pal_hardware_transmission_gazebo plugin only supports hardware_interface of type PositionJointInterface and EffortJointInterface!");
        ROS_ERROR_STREAM("The joint : "
                         << transmission.joints_[i].name_
                         << " has no hardware interface of type either PositionJointInterface or EffortJointInterface");
        return false;
      }
    }
  }

  // Simulation joints: All joints to control
  for (size_t i = 0; i < transmissions.size(); i++)
  {
    transmissions_data_[i].joint_names_.clear();
    for (const transmission_interface::JointInfo &joint_info : transmissions[i].joints_)
    {
      gazebo::physics::JointPtr joint = model->GetJoint(joint_info.name_);
      if (!joint)
      {
        ROS_ERROR_STREAM_NAMED("pal_hw_transmission_sim",
                               "This robot has a joint named \""
                                   << joint_info.name_ << "\" which is not in the gazebo model.");
        return false;
      }
      sim_joints_.push_back(joint);
      transmissions_data_[i].joint_names_.push_back(joint_info.name_);
      joint_names_.push_back(joint_info.name_);

      jnt_ctrl_mthd_.push_back(std::vector<JointControlMethod>());
      for (size_t j = 0; j < joint_info.hardware_interfaces_.size(); j++)
      {
        std::string hw_int = joint_info.hardware_interfaces_[j];
        ROS_INFO_STREAM("Joint " << joint_info.name_ << " has hardware interface " << hw_int);
        if (hw_int == "hardware_interface/EffortJointInterface")
        {
          jnt_ctrl_mthd_.back().push_back(JointControlMethod::EFFORT);
        }
        else if (hw_int == "hardware_interface/PositionJointInterface")
        {
          jnt_ctrl_mthd_.back().push_back(JointControlMethod::POSITION_PID);
        }
        //        else
        //        {
        //          ROS_WARN_STREAM_NAMED(
        //              "pal_hw_transmission_sim",
        //              "Joint " << joint_info.name_ << " of transmission " <<
        //              transmissions[i].name_
        //                       << " specifies multiple hardware interfaces. "
        //                       << "Currently the default robot hardware simulation
        //                       interface considers only the first entry");
        //          ROS_FATAL_STREAM_NAMED("pal_hw_transmission_sim",
        //                                 "No matching hardware interface found for '"
        //                                     << hw_int << "' while loading interfaces
        //                                     for "
        //                                     << joint_info.name_);
        //          return false;
        //        }
      }
    }
  }
  n_dof_ = sim_joints_.size();
  ROS_INFO_STREAM("Successfully retreived joint names!");

  // Get actuators count
  int actuators_count = 0;
  for (size_t i = 0; i < transmissions.size(); i++)
  {
    actuators_count += transmissions[i].actuators_.size();
  }

  // Raw data
  jnt_pos_.resize(n_dof_);
  jnt_vel_.resize(n_dof_);
  jnt_eff_.resize(n_dof_);
  act_pos_.resize(actuators_count);
  act_vel_.resize(actuators_count);
  act_eff_.resize(actuators_count);
  jnt_pos_cmd_.resize(n_dof_);
  jnt_eff_cmd_.resize(n_dof_);
  jnt_types_.resize(n_dof_);

  /// Retrieving max joint effort from urdf because values are not set in sim_joints_
  jnt_max_effort_.resize(n_dof_);
  for (size_t j = 0; j < n_dof_; ++j)
  {
    if (urdf->getJoint(sim_joints_[j]->GetName()) &&
        urdf->getJoint(sim_joints_[j]->GetName())->limits)
      jnt_max_effort_[j] = urdf->getJoint(sim_joints_[j]->GetName())->limits->effort;
    else
    {
      ROS_WARN_STREAM(sim_joints_[j]->GetName()
                      << " doesn't have effort limit, usuing default 10.0");
      jnt_max_effort_[j] = 10.0;
    }
  }

  /// Retrieving joint types from the URDF
  for (size_t i = 0; i < n_dof_; i++)
  {
    jnt_types_[i] = sim_joints_[i]->GetType();
  }

  // Hardware interfaces: joints
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_state_interface_.registerHandle(
        JointStateHandle(joint_names_[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));

    for (size_t j = 0; j < jnt_ctrl_mthd_[i].size(); j++)
    {
      if (jnt_ctrl_mthd_[i][j] == JointControlMethod::POSITION_PID)
      {
        jnt_pos_cmd_interface_.registerHandle(
            JointHandle(jnt_state_interface_.getHandle(joint_names_[i]), &jnt_pos_cmd_[i]));
        ROS_INFO_STREAM("Registered joint '" << joint_names_[i]
                                             << "' in the PositionJointInterface.");
      }
      else if (jnt_ctrl_mthd_[i][j] == JointControlMethod::EFFORT)
      {
        jnt_eff_cmd_interface_.registerHandle(
            JointHandle(jnt_state_interface_.getHandle(joint_names_[i]), &jnt_eff_cmd_[i]));
        ROS_INFO_STREAM("Registered joint '" << joint_names_[i]
                                             << "' in the EffortJointInterface.");
      }
    }

    act_state_interface_.registerHandle(
        ActuatorStateHandle(joint_names_[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));
  }

  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_cmd_interface_);
  registerInterface(&jnt_eff_cmd_interface_);

  // Define the actuator and joint data inside transmission data
  for (size_t i = 0; i < transmissions_data_.size(); i++)
  {
    for (const std::string &joint : transmissions_data_[i].joint_names_)
    {
      int k = getJointIndex(joint_names_, joint);
      if (k == -1)
      {
        ROS_ERROR_STREAM("Unable to find the joint : " << joint << " to update transimission data!");
        return false;
      }
      transmissions_data_[i].joint_data_.effort.push_back(&jnt_eff_[k]);
      transmissions_data_[i].joint_data_.velocity.push_back(&jnt_vel_[k]);
      transmissions_data_[i].joint_data_.position.push_back(&jnt_pos_[k]);
      transmissions_data_[i].joint_cmd_data_.position.push_back(&jnt_pos_cmd_[k]);
      transmissions_data_[i].joint_cmd_data_.effort.push_back(&jnt_eff_cmd_[k]);
    }
  }
  for (size_t i = 0; i < transmissions.size(); i++)
  {
    for (size_t k = 0; k < transmissions[i].actuators_.size(); k++)
    {
      transmissions_data_[i].actuator_data_.effort.push_back(&act_eff_[k + i]);
      transmissions_data_[i].actuator_data_.velocity.push_back(&act_vel_[k + i]);
      transmissions_data_[i].actuator_data_.position.push_back(&act_pos_[k + i]);
    }
  }

  // Transmission interface : Load the transmission interfce plugin
  transmission_classes_.resize(transmissions.size());
  for (size_t i = 0; i < transmissions.size(); i++)
  {
    try
    {
      transmission_class_loader_.reset(new TransmissionClassLoader(
          "transmission_interface", "transmission_interface::TransmissionLoader"));
      TransmissionLoaderPtr transmission_loader =
          transmission_class_loader_->createInstance(transmissions[i].type_);
      transmission_classes_[i] = transmission_loader->load(transmissions[i]);
      if (!transmission_classes_[i])
      {
        ROS_ERROR_STREAM_NAMED(
            "transmission_loader",
            "Problem loading the transmission type : " << transmissions[i].type_);
        return false;
      }
    }
    catch (pluginlib::LibraryLoadException &ex)
    {
      ROS_ERROR_STREAM_NAMED("transmission_loader",
                             "Failed to load transmission '"
                                 << transmissions[0].name_ << "'. Unsupported type '"
                                 << transmissions[0].type_ << "'.\n"
                                 << ex.what());
      return false;
    }
  }

  // Joint limits interface
  for (size_t i = 0; i < n_dof_; i++)
  {
    using namespace joint_limits_interface;
    std::shared_ptr<const urdf::Joint> urdf_joint = urdf->getJoint(joint_names_[i]);
    JointLimits limits;
    SoftJointLimits soft_limits;
    const bool hard_limits_status = joint_limits_interface::getJointLimits(urdf_joint, limits);
    const bool soft_limits_status =
        joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits);
    if (!soft_limits_status && !hard_limits_status)
    {
      ROS_WARN_STREAM("Joint limits are not found for joint '"
                      << joint_names_[i] << "' so not enforcing any limits!.");
      continue;
    }
    for (size_t j = 0; j < jnt_ctrl_mthd_.size(); j++)
    {
      JointHandle cmd_handle;

      if (jnt_ctrl_mthd_[i][j] == JointControlMethod::POSITION_PID)
      {
        cmd_handle = jnt_pos_cmd_interface_.getHandle(joint_names_[i]);
        if (soft_limits_status)
        {
          jnt_limits_interface_.registerHandle(
              PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));
        }
        else
        {
          jnt_sat_interface_.registerHandle(PositionJointSaturationHandle(cmd_handle, limits));
        }
      }
      else if (jnt_ctrl_mthd_[i][j] == JointControlMethod::EFFORT)
      {
        cmd_handle = jnt_eff_cmd_interface_.getHandle(joint_names_[i]);
        if (soft_limits_status)
        {
          eff_limits_interface_.registerHandle(
              EffortJointSoftLimitsHandle(cmd_handle, limits, soft_limits));
        }
        else
        {
          eff_sat_interface_.registerHandle(EffortJointSaturationHandle(cmd_handle, limits));
        }
      }
    }
    ROS_INFO_STREAM("Joint limits will be enforced for joint '" << joint_names_[i] << "'.");
  }

  // PID controllers
  pids_.resize(n_dof_);
  for (size_t i = 0; i < n_dof_; ++i)
  {
    for (size_t j = 0; j < jnt_ctrl_mthd_.size(); j++)
    {
      if (jnt_ctrl_mthd_[i][j] == JointControlMethod::EFFORT)
        continue;
      ros::NodeHandle joint_nh(nh, "gains/" + joint_names_[i]);
      if (!pids_[i].init(joint_nh))
      {
        ROS_ERROR_STREAM_NAMED("pids_loader",
                               "Unable to find the PIDs for the  joint "
                                   << joint_names_[i]
                                   << " in the namespace : " << joint_nh.getNamespace());
        return false;
      }
    }
  }

  ROS_INFO("Successfully initialized pal_hardware_gazebo/PalHardwareTransmissionGazebo!");
  return true;
}

void PalHardwareTransmissionGazebo::readSim(ros::Time time, ros::Duration period)
{
  // Read joint state and fill in the data
  for (unsigned int j = 0; j < n_dof_; ++j)
  {
    if (jnt_types_[j] == urdf::Joint::PRISMATIC)
    {
      jnt_pos_[j] = sim_joints_[j]->Position();
    }
    else
    {
      jnt_pos_[j] +=
          angles::shortest_angular_distance(jnt_pos_[j], sim_joints_[j]->Position());
    }
    jnt_vel_[j] = sim_joints_[j]->GetVelocity(0u);
    jnt_eff_[j] = sim_joints_[j]->GetForce(0u);
  }
}

bool PalHardwareTransmissionGazebo::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo> &start_list,
    const std::list<hardware_interface::ControllerInfo> &stop_list)
{
  std::map<std::string, std::string> joint_interface_map;

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    joint_interface_map.emplace(
        std::make_pair(joint_names_[i], std::string("hardware_interface::NoControl")));
  }

  for (auto it = start_list.begin(); it != start_list.end(); it++)
  {
    for (size_t i = 0; i < it->claimed_resources.size(); i++)
    {
      for (auto it2 = it->claimed_resources[i].resources.begin();
           it2 != it->claimed_resources[i].resources.end(); it2++)
      {
        joint_interface_map[*it2] = it->claimed_resources[i].hardware_interface;
      }
    }
  }


  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    ROS_INFO_STREAM("Receiving load transmission "
                    << joint_names_[i] << " hardware interface type "
                    << joint_interface_map.at(joint_names_[i]));
    current_jnt_ctrl_mthd_.push_back(joint_interface_map.at(joint_names_[i]));
  }

  return true;
}

void PalHardwareTransmissionGazebo::writeSim(ros::Time time, ros::Duration period)
{
  // Enforce joint limits based on URDF
  jnt_sat_interface_.enforceLimits(period);
  jnt_limits_interface_.enforceLimits(period);
  eff_sat_interface_.enforceLimits(period);
  eff_limits_interface_.enforceLimits(period);

  // Compute the final joint command input
  // The below iteration make sures that the position in the custom transmissions are
  // limited as per the defined boundaries
  for (size_t i = 0; i < transmissions_data_.size(); i++)
  {
    transmission_classes_[i]->jointToActuatorPosition(
        transmissions_data_[i].joint_cmd_data_, transmissions_data_[i].actuator_data_);

    transmission_classes_[i]->actuatorToJointPosition(
        transmissions_data_[i].actuator_data_, transmissions_data_[i].joint_cmd_data_);

    transmission_classes_[i]->jointToActuatorEffort(transmissions_data_[i].joint_cmd_data_,
                                                    transmissions_data_[i].actuator_data_);

    transmission_classes_[i]->actuatorToJointEffort(transmissions_data_[i].actuator_data_,
                                                    transmissions_data_[i].joint_cmd_data_);
  }

  if(current_jnt_ctrl_mthd_.size() < 1)
      return;

  // Compute and send effort command
  for (unsigned int j = 0; j < n_dof_; ++j)
  {
    if (current_jnt_ctrl_mthd_[j] == "hardware_interface::EffortJointInterface")
    {
        sim_joints_[j]->SetForce(0, jnt_eff_cmd_[j]);
    }
    else if (current_jnt_ctrl_mthd_[j] == "hardware_interface::PositionJointInterface")
    {
      // Assumes jnt_pos_ contains most recent value from the readSim iteration
      double error;
      switch (jnt_types_[j])
      {
        case urdf::Joint::REVOLUTE:
          error = angles::shortest_angular_distance(jnt_pos_[j], jnt_pos_cmd_[j]);
          break;
        default:
          error = jnt_pos_cmd_[j] - jnt_pos_[j];
      }
      const double effort = pids_[j].computeCommand(error, period);

      const double max_effort = jnt_max_effort_[j];
      const double min_effort = -max_effort;
      double effort_modified = (effort - max_effort) > 1e-4 ? max_effort : effort;
      effort_modified = effort_modified - min_effort < -1e-4 ? min_effort : effort_modified;

      sim_joints_[j]->SetForce(0u, effort_modified);
    }
    else
    {
      // Assumes jnt_pos_ contains most recent value from the readSim iteration
      double error;
      switch (jnt_types_[j])
      {
        case urdf::Joint::REVOLUTE:
          error = angles::shortest_angular_distance(jnt_pos_[j], jnt_pos_cmd_[j]);
          break;
        default:
          error = jnt_pos_cmd_[j] - jnt_pos_[j];
      }
      const double effort = pids_[j].computeCommand(error, period);

      const double max_effort = jnt_max_effort_[j];
      const double min_effort = -max_effort;
      double effort_modified = (effort - max_effort) > 1e-4 ? max_effort : effort;
      effort_modified = effort_modified - min_effort < -1e-4 ? min_effort : effort_modified;
      sim_joints_[j]->SetForce(0u, effort_modified);
    }
  }
}

}  // pal_hardware_gazebo

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::PalHardwareTransmissionGazebo,
                       gazebo_ros_control::RobotHWSim)

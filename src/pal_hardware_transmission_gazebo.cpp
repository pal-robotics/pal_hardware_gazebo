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
  ROS_DEBUG("Parsed robot description");

  // Cleanup
  sim_joints_.clear();
  jnt_pos_.clear();
  jnt_vel_.clear();
  jnt_eff_.clear();
  act_pos_.clear();
  act_vel_.clear();
  act_eff_.clear();
  jnt_pos_cmd_.clear();
  joint_names_.clear();
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
        if (hw_inf == "hardware_interface/PositionJointInterface")
        {
          interface_status = true;
          break;
        }
      }
      if (!interface_status)
      {
        ROS_WARN_STREAM("The pal_hardware_transmission_gazebo plugin only supports hardware_interface of type PositionJointInterface!");
        ROS_ERROR_STREAM("The joint : "
                         << transmission.joints_[i].name_
                         << " has no hardware interface of type PositionJointInterface");
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
      sim_joints_.push_back(model->GetJoint(joint_info.name_));
      transmissions_data_[i].joint_names_.push_back(joint_info.name_);
      joint_names_.push_back(joint_info.name_);
    }
  }
  n_dof_ = sim_joints_.size();
  ROS_DEBUG_STREAM("Successfully retreived joint names!");

  // Raw data
  jnt_pos_.resize(n_dof_);
  jnt_vel_.resize(n_dof_);
  jnt_eff_.resize(n_dof_);
  act_pos_.resize(n_dof_);
  act_vel_.resize(n_dof_);
  act_eff_.resize(n_dof_);
  jnt_pos_cmd_.resize(n_dof_);

  /// Retrieving max joint effort from urdf because values are not set in sim_joints_
  jnt_max_effort_.resize(n_dof_);
  for (size_t j = 0; j < n_dof_; ++j)
  {
    if (urdf->getJoint(sim_joints_[j]->GetName())->limits)
      jnt_max_effort_[j] = urdf->getJoint(sim_joints_[j]->GetName())->limits->effort;
    else
    {
      ROS_WARN_STREAM(sim_joints_[j]->GetName()
                      << " doesn't have effort limit, usuing default 10.0");
      jnt_max_effort_[j] = 10.0;
    }
  }

  // Hardware interfaces: joints
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_state_interface_.registerHandle(
        JointStateHandle(joint_names_[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));
    jnt_pos_cmd_interface_.registerHandle(
        JointHandle(jnt_state_interface_.getHandle(joint_names_[i]), &jnt_pos_cmd_[i]));
    ROS_DEBUG_STREAM("Registered joint '" << joint_names_[i] << "' in the PositionJointInterface.");

    act_state_interface_.registerHandle(
        ActuatorStateHandle(joint_names_[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_cmd_interface_);

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
      transmissions_data_[i].actuator_data_.effort.push_back(&act_eff_[k]);
      transmissions_data_[i].actuator_data_.velocity.push_back(&act_vel_[k]);
      transmissions_data_[i].actuator_data_.position.push_back(&act_pos_[k]);
      transmissions_data_[i].joint_cmd_data_.position.push_back(&jnt_pos_cmd_[k]);
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
      typedef boost::shared_ptr<transmission_interface::TransmissionLoader> TransmissionLoaderPtr;
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
  std::vector<std::string> cmd_handle_names = jnt_pos_cmd_interface_.getNames();
  for (unsigned int i = 0; i < cmd_handle_names.size(); ++i)
  {
    JointHandle cmd_handle = jnt_pos_cmd_interface_.getHandle(cmd_handle_names[i]);
    const std::string name = cmd_handle.getName();

    using namespace joint_limits_interface;
    std::shared_ptr<const urdf::Joint> urdf_joint = urdf->getJoint(name);
    JointLimits limits;
    SoftJointLimits soft_limits;
    if (!joint_limits_interface::getJointLimits(urdf_joint, limits) ||
        !joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
    {
      ROS_WARN_STREAM("Joint limits won't be enforced for joint '" << name << "'.");
      continue;
    }
    jnt_limits_interface_.registerHandle(
        PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));

    ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << name << "'.");
  }

  // PID controllers
  pids_.resize(n_dof_);
  for (size_t i = 0; i < n_dof_; ++i)
  {
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

  ROS_INFO("Successfully initialized pal_hardware_gazebo/PalHardwareTransmissionGazebo!");
  return true;
}

void PalHardwareTransmissionGazebo::readSim(ros::Time time, ros::Duration period)
{
  // Read joint state and fill in the data
  for (unsigned int j = 0; j < n_dof_; ++j)
  {
    jnt_pos_[j] += angles::shortest_angular_distance(jnt_pos_[j], sim_joints_[j]->Position());
    jnt_vel_[j] = sim_joints_[j]->GetVelocity(0u);
    jnt_eff_[j] = sim_joints_[j]->GetForce(0u);
  }
}

void PalHardwareTransmissionGazebo::writeSim(ros::Time time, ros::Duration period)
{
  // Enforce joint limits based on URDF
  jnt_limits_interface_.enforceLimits(period);

  // Compute the final joint command input
  // The below iteration make sures that the position in the custom transmissions are
  // limited as per the defined boundaries
  for (size_t i = 0; i < transmissions_data_.size(); i++)
  {
    transmission_classes_[i]->jointToActuatorPosition(
        transmissions_data_[i].joint_cmd_data_, transmissions_data_[i].actuator_data_);

    transmission_classes_[i]->actuatorToJointPosition(
        transmissions_data_[i].actuator_data_, transmissions_data_[i].joint_cmd_data_);
  }

  // Compute and send effort command
  for (unsigned int j = 0; j < n_dof_; ++j)
  {
    // Assumes jnt_pos_ contains most recent value from the readSim iteration
    const double error = jnt_pos_cmd_[j] - jnt_pos_[j];
    const double effort = pids_[j].computeCommand(error, period);

    const double max_effort = jnt_max_effort_[j];
    const double min_effort = -max_effort;
    double effort_modified = (effort - max_effort) > 1e-4 ? max_effort : effort;
    effort_modified = effort_modified - min_effort < -1e-4 ? min_effort : effort_modified;

    sim_joints_[j]->SetForce(0u, effort_modified);
  }
}

}  // pal_hardware_gazebo

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::PalHardwareTransmissionGazebo,
                       gazebo_ros_control::RobotHWSim)

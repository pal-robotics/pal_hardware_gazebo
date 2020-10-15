/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef PAL_HARDWARE_TRANSMISSION_GAZEBO_H
#define PAL_HARDWARE_TRANSMISSION_GAZEBO_H
/** \author Sai Kishor Kothakota **/

#include <control_toolbox/pid.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <gazebo_ros_control/robot_hw_sim.h>

#include <gazebo/physics/physics.hh>
#include <transmission_interface/transmission_interface_loader.h>

namespace gazebo_ros_control
{
class TransmissionData
{
public:
  TransmissionData()
  {
  }

  ~TransmissionData()
  {
  }

  std::vector<std::string> joint_names_;
  transmission_interface::ActuatorData actuator_data_;
  transmission_interface::JointData joint_data_;
  transmission_interface::JointData joint_cmd_data_;
};

class PalHardwareTransmissionGazebo : public gazebo_ros_control::RobotHWSim
{
public:
  PalHardwareTransmissionGazebo();

  // Simulation-specific
  bool initSim(const std::string& robot_ns, ros::NodeHandle nh,
               gazebo::physics::ModelPtr model, const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions);
  void readSim(ros::Time time, ros::Duration period);
  void writeSim(ros::Time time, ros::Duration period);

  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list) override final;

protected:
  // Methods used to control a joint.
  enum JointControlMethod
  {
    EFFORT,
    POSITION_PID
  };

private:
  // Raw data
  unsigned int n_dof_;

  std::vector<unsigned int> jnt_types_;
  std::vector<double> jnt_pos_;
  std::vector<double> jnt_vel_;
  std::vector<double> jnt_eff_;
  std::vector<double> act_pos_;
  std::vector<double> act_vel_;
  std::vector<double> act_eff_;
  std::vector<std::string> joint_names_;

  std::vector<double> jnt_pos_cmd_;
  std::vector<double> jnt_eff_cmd_;
  std::vector<double> jnt_max_effort_;

  // Control Method info
  std::vector<std::vector<JointControlMethod>> jnt_ctrl_mthd_;
  std::vector<std::string> current_jnt_ctrl_mthd_;

  // Simulation-specific
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  // Hardware interface: joints
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_cmd_interface_;
  hardware_interface::ActuatorStateInterface act_state_interface_;
  hardware_interface::EffortJointInterface jnt_eff_cmd_interface_;

  // Joint limits interface
  joint_limits_interface::PositionJointSaturationInterface jnt_sat_interface_;
  joint_limits_interface::EffortJointSaturationInterface eff_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface eff_limits_interface_;

  transmission_interface::RobotTransmissions robot_transmissions_;
  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;

  // Transmission interface
  typedef boost::shared_ptr<transmission_interface::Transmission> TransmissionPtr;
  typedef boost::shared_ptr<transmission_interface::TransmissionLoader> TransmissionLoaderPtr;
  typedef pluginlib::ClassLoader<transmission_interface::TransmissionLoader> TransmissionClassLoader;
  typedef boost::shared_ptr<TransmissionClassLoader> TransmissionClassLoaderPtr;
  TransmissionClassLoaderPtr transmission_class_loader_;
  std::vector<TransmissionPtr> transmission_classes_;
  std::vector<TransmissionData> transmissions_data_;

  // PID controllers
  std::vector<control_toolbox::Pid> pids_;
};
}

#endif  // PAL_HARDWARE_TRANSMISSION_GAZEBO_H

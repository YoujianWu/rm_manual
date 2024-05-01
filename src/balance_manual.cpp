//
// Created by yuchen on 2023/4/3.
//

#include "rm_manual/balance_manual.h"

namespace rm_manual
{
BalanceManual::BalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalShooterCoverManual(nh, nh_referee)
{
  ros::NodeHandle balance_nh(nh, "balance");
  balance_cmd_sender_ = new rm_common::BalanceCommandSender(balance_nh);
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);

  nh.param("flank_frame", flank_frame_, std::string("flank_frame"));
  nh.param("reverse_frame", reverse_frame_, std::string("yaw_reverse_frame"));
  nh.param("balance_dangerous_angle", balance_dangerous_angle_, 0.3);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("chassis_calibration", rpc_value);
  chassis_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);

  is_balance_ = true;
  state_sub_ = balance_nh.subscribe<rm_msgs::BalanceState>("/state", 1, &BalanceManual::balanceStateCallback, this);
  jump_pub_ = balance_nh.advertise<std_msgs::Bool>("/controllers/legged_balance_controller/jump_command", 1);
  leg_length_pub_ = balance_nh.advertise<std_msgs::Float64>("/controllers/legged_balance_controller/leg_command", 1);
  x_event_.setRising(boost::bind(&BalanceManual::xPress, this));
  g_event_.setRising(boost::bind(&BalanceManual::gPress, this));
  v_event_.setRising(boost::bind(&BalanceManual::vPress, this));
  auto_fallen_event_.setActiveHigh(boost::bind(&BalanceManual::modeFallen, this, _1));
  auto_fallen_event_.setDelayTriggered(boost::bind(&BalanceManual::modeNormalize, this), 1.5, true);
  ctrl_x_event_.setRising(boost::bind(&BalanceManual::ctrlXPress, this));
  ctrl_f_event_.setRising(boost::bind(&BalanceManual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&BalanceManual::ctrlGPress, this));
}

void BalanceManual::run()
{
  ChassisGimbalShooterCoverManual::run();
  chassis_calibration_->update(ros::Time::now());
}

void BalanceManual::sendCommand(const ros::Time& time)
{
  if (flank_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = flank_frame_;
  else if (reverse_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = reverse_frame_;
  else
    chassis_cmd_sender_->getMsg()->follow_source_frame = "yaw";

  if (supply_)
  {
    cover_close_ = false;
    cover_command_sender_->on();
  }
  else
  {
    cover_close_ = true;
    cover_command_sender_->off();
  }

  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
  balance_cmd_sender_->sendCommand(time);
}

void BalanceManual::checkReferee()
{
  ChassisGimbalShooterCoverManual::checkReferee();
  chassis_power_on_event_.update(chassis_output_on_);
}

void BalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  v_event_.update(dbus_data->key_v && !dbus_data->key_ctrl);
  ctrl_x_event_.update(dbus_data->key_ctrl && dbus_data->key_x);
  ctrl_f_event_.update(dbus_data->key_ctrl && dbus_data->key_f);
  ctrl_g_event_.update(dbus_data->key_ctrl && dbus_data->key_g);
}

void BalanceManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::updateRc(dbus_data);
  if (std::abs(dbus_data->ch_r_x) > 0.5 && std::abs(dbus_data->ch_r_x) > std::abs(dbus_data->ch_r_y))
    flank_ = true;
  else if (std::abs(dbus_data->ch_r_y) > 0.5 && std::abs(dbus_data->ch_r_y) > std::abs(dbus_data->ch_r_x))
    flank_ = false;
}

void BalanceManual::rightSwitchDownRise()
{
  ChassisGimbalShooterCoverManual::rightSwitchDownRise();
  state_ = RC;
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void BalanceManual::rightSwitchMidRise()
{
  ChassisGimbalShooterCoverManual::rightSwitchMidRise();
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::ctrlZPress()
{
  ChassisGimbalShooterCoverManual::ctrlZPress();
  if (supply_)
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  }
  else
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}

void BalanceManual::shiftRelease()
{
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  std_msgs::Float64 msg;
  msg.data = 0.18;
  leg_length_pub_.publish(msg);
}

void BalanceManual::shiftPress()
{
  ChassisGimbalShooterCoverManual::shiftPress();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::UP_SLOPE);
  std_msgs::Float64 msg;
  msg.data = 0.1;
  leg_length_pub_.publish(msg);
  chassis_cmd_sender_->updateSafetyPower(220);
}

void BalanceManual::vPress()
{
  chassis_cmd_sender_->updateSafetyPower(220);
}

void BalanceManual::bPress()
{
  ChassisGimbalShooterCoverManual::bPress();
  chassis_cmd_sender_->updateSafetyPower(220);
}

void BalanceManual::bRelease()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::wPress()
{
  if (flank_)
    flank_ = !flank_;
  if (!supply_)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  if (is_gyro_)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
  }
  ChassisGimbalShooterCoverManual::wPress();
}

void BalanceManual::wPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::wPressing();
  if (supply_)
    vel_cmd_sender_->setLinearXVel(x_scale_ * 0.4);
}

void BalanceManual::sPress()
{
  if (flank_)
    flank_ = !flank_;
  if (!supply_)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  if (is_gyro_)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
  }
  ChassisGimbalShooterCoverManual::sPress();
}

void BalanceManual::sPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::sPressing();
  if (supply_)
    vel_cmd_sender_->setLinearXVel(x_scale_ * 0.4);
}

void BalanceManual::aPress()
{
  if (!flank_)
    flank_ = !flank_;
  if (is_gyro_)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
  }
  ChassisGimbalShooterCoverManual::aPress();
}

void BalanceManual::aPressing()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::aPressing();
  if (supply_)
    vel_cmd_sender_->setLinearYVel(y_scale_ * 0.4);
}

void BalanceManual::dPress()
{
  if (!flank_)
    flank_ = !flank_;
  if (is_gyro_)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
  }
  ChassisGimbalShooterCoverManual::dPress();
}

void BalanceManual::dPressing()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::dPressing();
  if (supply_)
    vel_cmd_sender_->setLinearYVel(y_scale_ * 0.4);
}

void BalanceManual::cPress()
{
  if (is_gyro_)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
    is_gyro_ = false;
  }
  else
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    is_gyro_ = true;
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(1.0);
  }
}

void BalanceManual::ctrlXPress()
{
  if (balance_cmd_sender_->getMsg()->data == rm_msgs::BalanceState::NORMAL)
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  else
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void BalanceManual::ctrlFPress()
{
  std_msgs::Bool msg;
  msg.data = true;
  jump_pub_.publish(msg);
}

void BalanceManual::ctrlGPress()
{
  std_msgs::Float64 msg;
  if (!stretch_)
  {
    msg.data = 0.25;
    stretch_ = true;
  }
  else
  {
    msg.data = 0.2;
    stretch_ = false;
  }

  leg_length_pub_.publish(msg);
}

void BalanceManual::balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg)
{
  if ((ros::Time::now() - msg->header.stamp).toSec() < 0.2)
  {
    if (std::abs(msg->theta) > balance_dangerous_angle_)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    if (msg->mode == rm_msgs::BalanceState::NORMAL)
      auto_fallen_event_.update(std::abs(msg->theta) > 0.42 && std::abs(msg->x_dot) > 1.5 &&
                                vel_cmd_sender_->getMsg()->linear.x == 0 && vel_cmd_sender_->getMsg()->linear.y == 0 &&
                                vel_cmd_sender_->getMsg()->angular.z == 0);
  }
}

void BalanceManual::chassisOutputOn()
{
  ChassisGimbalShooterCoverManual::chassisOutputOn();
  chassis_calibration_->reset();
}

void BalanceManual::remoteControlTurnOff()
{
  ChassisGimbalShooterCoverManual::remoteControlTurnOff();
  chassis_calibration_->stop();
}

void BalanceManual::remoteControlTurnOn()
{
  ChassisGimbalShooterCoverManual::remoteControlTurnOn();
  chassis_calibration_->stopController();
}

void BalanceManual::modeNormalize()
{
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  ROS_INFO("mode normalize");
}

void BalanceManual::modeFallen(ros::Duration duration)
{
  if (duration.toSec() > 0.3)
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
    ROS_INFO("mode fallen");
  }
}
}  // namespace rm_manual

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
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("chassis_calibration", rpc_value);
  chassis_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);

  is_balance_ = true;
  jump_pub_ = balance_nh.advertise<std_msgs::Bool>("/controllers/legged_balance_controller/jump_command", 1);
  leg_length_pub_ = balance_nh.advertise<std_msgs::Float64>("/controllers/legged_balance_controller/leg_command", 1);
  x_event_.setRising(boost::bind(&BalanceManual::xPress, this));
  g_event_.setRising(boost::bind(&BalanceManual::gPress, this));
  v_event_.setRising(boost::bind(&BalanceManual::vPress, this));
  ctrl_x_event_.setRising(boost::bind(&BalanceManual::ctrlXPress, this));
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
  if (dbus_data->wheel && dbus_data->s_l == 3)
  {
    std_msgs::Float64 msg;
    msg.data = abs(dbus_data->wheel) * 0.3 > 0.1 ? abs(dbus_data->wheel) * 0.3 : 0.15;
    leg_length_pub_.publish(msg);
    vel_cmd_sender_->setAngularZVel(0.0);
  }
  else
  {
    vel_cmd_sender_->setAngularZVel(1.0);
  }
  if (!is_gyro_)
  {  // Capacitor enter fast charge when chassis stop.
    if (!dbus_data->wheel && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
            0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    else if (chassis_power_ < 6.0 && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
  else
  {
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
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
  // std_msgs::Float64 msg;
  // msg.data = 0.18;
  // leg_length_pub_.publish(msg);
}

void BalanceManual::shiftPress()
{
  ChassisGimbalShooterCoverManual::shiftPress();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::UP_SLOPE);
  //  std_msgs::Float64 msg;
  //  msg.data = 0.1;
  //  leg_length_pub_.publish(msg);
  chassis_cmd_sender_->updateSafetyPower(220);
}

void BalanceManual::vPress()
{
  std_msgs::Bool msg;
  msg.data = true;
  jump_pub_.publish(msg);
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
  //  if (is_gyro_)
  //  {
  //    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  //    vel_cmd_sender_->setAngularZVel(0.0);
  //  }
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
  //  if (is_gyro_)
  //  {
  //    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  //    vel_cmd_sender_->setAngularZVel(0.0);
  //  }
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
  //  if (is_gyro_)
  //  {
  //    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  //    vel_cmd_sender_->setAngularZVel(0.0);
  //  }
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
  //  if (is_gyro_)
  //  {
  //    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  //    vel_cmd_sender_->setAngularZVel(0.0);
  //  }
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
}  // namespace rm_manual

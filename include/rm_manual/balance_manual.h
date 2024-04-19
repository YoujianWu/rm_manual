//
// Created by yuchen on 2023/4/3.
//

#pragma once

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual
{
class BalanceManual : public ChassisGimbalShooterCoverManual
{
public:
  BalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

protected:
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void remoteControlTurnOff() override;
  void remoteControlTurnOn() override;
  void wPress() override;
  void sPress() override;
  void aPress() override;
  void dPress() override;
  void cPress() override;
  void shiftPress() override;
  void shiftRelease() override;
  void wPressing() override;
  void aPressing() override;
  void sPressing() override;
  void dPressing() override;
  void bPress() override;
  void vPress() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;

  void chassisOutputOn() override;
  void sendCommand(const ros::Time& time) override;
  void checkReferee() override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void ctrlXPress();
  void ctrlFPress();
  void ctrlGPress();
  void modeFallen(ros::Duration duration);
  void modeNormalize();
  rm_common::BalanceCommandSender* balance_cmd_sender_{};

private:
  void balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg);

  ros::Subscriber state_sub_;
  ros::Publisher jump_pub_;
  ros::Publisher leg_length_pub_;
  bool stretch_ = false;
  double balance_dangerous_angle_;

  bool flank_ = false, reverse_ = false;
  std::string flank_frame_, reverse_frame_;
  rm_common::CalibrationQueue* chassis_calibration_;

  InputEvent v_event_, ctrl_x_event_, ctrl_f_event_, ctrl_g_event_, auto_fallen_event_;
};
}  // namespace rm_manual

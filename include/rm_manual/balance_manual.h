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
  void bRelease() override;
  void vPress() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;

  void chassisOutputOn() override;
  void robotRevive() override;
  void sendCommand(const ros::Time& time) override;
  void checkReferee() override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void ctrlXPress();
  void ctrlGPress();
  rm_common::BalanceCommandSender* balance_cmd_sender_{};

private:
  ros::Publisher jump_pub_;
  ros::Publisher leg_length_pub_;
  bool stretch_ = false;

  bool flank_ = false, reverse_ = false;
  std::string flank_frame_, reverse_frame_;
  rm_common::CalibrationQueue* chassis_calibration_;

  InputEvent v_event_, ctrl_x_event_, ctrl_g_event_;
};
}  // namespace rm_manual

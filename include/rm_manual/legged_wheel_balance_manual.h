//
// Created by kook on 9/28/24.
//

#pragma once

#include "rm_manual/balance_manual.h"

namespace rm_manual
{
class LeggedWheelBalanceManual : public BalanceManual
{
public:
  LeggedWheelBalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void shiftPress() override;
  void shiftRelease() override;
  void bPress() override;
  void bRelease() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchDownRise() override;
  void ctrlPress();
  void ctrlRelease();

  void sendCommand(const ros::Time& time) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void ctrlGPress();
  rm_common::LegCommandSender* legCommandSender_{};

private:
  bool stretch_ = false, stretching_ = false, is_increasing_length_ = false;
  InputEvent ctrl_event_, ctrl_g_event_;
  ros::Subscriber unstick_sub_;
  void unstickCallback(const std_msgs::BoolConstPtr& msg);
};
}  // namespace rm_manual

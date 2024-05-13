/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne_gui/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <gtkmm.h>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <robotem_rovne/srv/angular_target.hpp>

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/angular/angular.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::mm;
using mp_units::si::unit_symbols::s;
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;

namespace t07
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node(Glib::RefPtr<Gtk::Application> gtk_app,
        Glib::RefPtr<Gtk::Builder> gtk_builder);
  ~Node();

private:
  Glib::RefPtr<Gtk::Application> _gtk_app;
  Glib::RefPtr<Gtk::Builder> _gtk_builder;
  Glib::Dispatcher _gtk_dispatcher;
  std::thread _gtk_thread;
  void init_gtk();
  void gtk_on_notification_from_worker_thread();

  void btn_start_pressed();
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _req_start_service_client;
  void init_req_start_service_client();
  void request_start();

  void btn_stop_pressed();
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _req_stop_service_client;
  void init_req_stop_service_client();
  void request_stop();

  void btn_set_pressed();
  rclcpp::Client<robotem_rovne::srv::AngularTarget>::SharedPtr _req_set_target_angle_service_client;
  void init_req_set_target_angle_service_client();
  void request_set_target_angle(float const target_angle_rad);

  rclcpp::QoS _imu_qos_profile;
  rclcpp::SubscriptionOptions _imu_sub_options;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
  std::mutex _yaw_mtx;
  double _yaw_actual;
  void init_imu_sub();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */

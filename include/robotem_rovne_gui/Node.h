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

#include <std_msgs/msg/float32.hpp>

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
  std::thread _gtk_thread;

  void btn_start_pressed();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */

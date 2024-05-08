/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne_gui/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <robotem_rovne_gui/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace t07
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node(Glib::RefPtr<Gtk::Application> gtk_app,
           Glib::RefPtr<Gtk::Builder> gtk_builder)
: rclcpp::Node("robotem_rovne_gui_node")
, _gtk_app{gtk_app}
, _gtk_builder{gtk_builder}
, _gtk_thread{}
{
  Gtk::Button * btn_start = nullptr;
  _gtk_builder->get_widget("btn_start", btn_start);
  btn_start->signal_clicked().connect(sigc::mem_fun(*this, &Node::btn_start_pressed));

  Gtk::Button * btn_stop = nullptr;
  _gtk_builder->get_widget("btn_stop", btn_stop);
  btn_stop->signal_clicked().connect(sigc::mem_fun(*this, &Node::btn_stop_pressed));

  Gtk::Button * btn_set = nullptr;
  _gtk_builder->get_widget("btn_set", btn_set);
  btn_set->signal_clicked().connect(sigc::mem_fun(*this, &Node::btn_set_pressed));

  _gtk_thread = std::thread(
    [this]()
    {
      Gtk::Window * window = nullptr;
      _gtk_builder->get_widget("robotem_rovne_gui_window", window);
      _gtk_app->run(*window);
      rclcpp::shutdown();
    });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  _gtk_app->quit();
  _gtk_thread.join();
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::btn_start_pressed()
{
  RCLCPP_INFO(get_logger(), "btn_start_pressed");
}

void Node::btn_stop_pressed()
{
  RCLCPP_INFO(get_logger(), "btn_stop_pressed");
}

void Node::btn_set_pressed()
{
  Gtk::Entry * entry_target_angle = nullptr;
  _gtk_builder->get_widget("entry_target_angle", entry_target_angle);

  std::stringstream angle_ss;
  angle_ss << entry_target_angle->get_text().c_str();
  float angle = 0.f;
  angle_ss >> angle;

  RCLCPP_INFO(get_logger(), "btn_set_pressed: angle = %0.2f", angle);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */

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

  _gtk_thread = std::thread(
    [this]()
    {
      Gtk::Window * window = nullptr;
      _gtk_builder->get_widget("robotem_rovne_gui_window", window);
      return _gtk_app->run(*window);
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

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */

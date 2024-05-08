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
  init_gtk();

  init_req_start_service_client();
  init_req_stop_service_client();
  init_req_set_target_angle_service_client();

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

void Node::init_gtk()
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
}

void Node::btn_start_pressed()
{
  RCLCPP_DEBUG(get_logger(), "btn_start_pressed");
  request_start();
}

void Node::init_req_start_service_client()
{
  _req_start_service_client = create_client<std_srvs::srv::Empty>("cmd_robot/start");
}

void Node::request_start()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto onResponseCallback = [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /* response */)
  {
    RCLCPP_INFO(get_logger(), "start request sent and confirmed.");
  };
  auto future_response = _req_start_service_client->async_send_request(request, onResponseCallback);
}

void Node::btn_stop_pressed()
{
  RCLCPP_DEBUG(get_logger(), "btn_stop_pressed");
  request_stop();
}

void Node::init_req_stop_service_client()
{
  _req_stop_service_client = create_client<std_srvs::srv::Empty>("cmd_robot/stop");
}

void Node::request_stop()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto onResponseCallback = [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /* response */)
  {
    RCLCPP_INFO(get_logger(), "stop request sent and confirmed.");
  };
  auto future_response = _req_stop_service_client->async_send_request(request, onResponseCallback);
}

void Node::btn_set_pressed()
{
  Gtk::Entry * entry_target_angle = nullptr;
  _gtk_builder->get_widget("entry_target_angle", entry_target_angle);

  std::stringstream angle_ss;
  angle_ss << entry_target_angle->get_text().c_str();
  float angle = 0.f;
  angle_ss >> angle;

  RCLCPP_DEBUG(get_logger(), "btn_set_pressed: angle = %0.2f", angle);
  request_set_target_angle(angle);
}

void Node::init_req_set_target_angle_service_client()
{
  _req_set_target_angle_service_client = create_client<robotem_rovne::srv::AngularTarget>("cmd_robot/set_angular_target");
}

void Node::request_set_target_angle(float const target_angle_rad)
{
  auto request = std::make_shared<robotem_rovne::srv::AngularTarget::Request>();
  request->target_angle_rad = target_angle_rad;
  auto onResponseCallback = [this](rclcpp::Client<robotem_rovne::srv::AngularTarget>::SharedFuture /* response */)
  {
    RCLCPP_INFO(get_logger(), "set target angle request sent and confirmed.");
  };
  auto future_response = _req_set_target_angle_service_client->async_send_request(request, onResponseCallback);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */

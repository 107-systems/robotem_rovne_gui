/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne_gui/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <robotem_rovne_gui/Node.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
, _imu_qos_profile{rclcpp::KeepLast(10), rmw_qos_profile_sensor_data}
{
  init_gtk();

  init_req_start_service_client();
  init_req_stop_service_client();
  init_req_set_target_angle_service_client();

  init_imu_sub();

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

  _gtk_dispatcher.connect(sigc::mem_fun(*this, &Node::gtk_on_notification_from_worker_thread));
}

void Node::gtk_on_notification_from_worker_thread()
{
  char yaw_actual_buf[32] = {0};
  {
    std::lock_guard<std::mutex> lock(_yaw_mtx);
    snprintf(yaw_actual_buf, sizeof(yaw_actual_buf), "%0.2f", _yaw_actual);
  }

  Gtk::Label * label_yaw_actual = nullptr;
  _gtk_builder->get_widget("label_yaw_actual", label_yaw_actual);
  assert(label_yaw_actual);
  label_yaw_actual->set_label(std::string(yaw_actual_buf));
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
  Gtk::Entry * entry_yaw_target = nullptr;
  _gtk_builder->get_widget("entry_yaw_target", entry_yaw_target);

  std::stringstream angle_ss;
  angle_ss << entry_yaw_target->get_text().c_str();
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

void Node::init_imu_sub()
{
  auto const imu_topic = std::string("imu");
  auto const imu_topic_deadline = std::chrono::milliseconds(100);
  auto const imu_topic_liveliness_lease_duration = std::chrono::milliseconds(1000);

  _imu_qos_profile.deadline(imu_topic_deadline);
  _imu_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _imu_qos_profile.liveliness_lease_duration(imu_topic_liveliness_lease_duration);

  _imu_sub_options.event_callbacks.deadline_callback =
    [this, imu_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            imu_topic.c_str(), event.total_count, event.total_count_change);
    };

  _imu_sub_options.event_callbacks.liveliness_callback =
    [this, imu_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", imu_topic.c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", imu_topic.c_str());
      }
    };

  _imu_sub = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    _imu_qos_profile,
    [this](sensor_msgs::msg::Imu::SharedPtr const msg)
    {
      {
        std::lock_guard<std::mutex> lock(_yaw_mtx);
        _yaw_actual = static_cast<double>(tf2::getYaw(msg->orientation));
      }

      _gtk_dispatcher.emit();

      RCLCPP_INFO(get_logger(),
                  "IMU Pose (theta) | (x,y,z,w): %0.2f | %0.2f %0.2f %0.2f %0.2f",
                  _yaw_actual,
                  msg->orientation.x,
                  msg->orientation.y,
                  msg->orientation.z,
                  msg->orientation.w);
    },
    _imu_sub_options);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */

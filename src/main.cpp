/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne_gui/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <robotem_rovne_gui/Node.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char ** argv) try
{
  rclcpp::init(argc, argv);

  Glib::RefPtr<Gtk::Application> gtk_app = Gtk::Application::create(argc, argv, "");
  Glib::RefPtr<Gtk::Builder> gtk_builder = Gtk::Builder::create();
  gtk_builder->add_from_file("install/robotem_rovne_gui/share/robotem_rovne_gui/glade/robotem_rovne_gui.glade");

  auto node = std::make_shared<t07::Node>(gtk_app, gtk_builder);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
catch (std::runtime_error const & err)
{
  RCLCPP_ERROR(rclcpp::get_logger(""), "Exception (std::runtime_error) caught: %s\nTerminating ...", err.what());
  return EXIT_FAILURE;
}
catch(const Glib::FileError & err)
{
  RCLCPP_ERROR(rclcpp::get_logger(""), "Exception (Glib::FileError) caught: %s\nTerminating ...", err.what().c_str());
  return EXIT_FAILURE;
}
catch(const Glib::MarkupError & err)
{
  RCLCPP_ERROR(rclcpp::get_logger(""), "Exception (Glib::MarkupError) caught: %s\nTerminating ...", err.what().c_str());
  return EXIT_FAILURE;
}
catch(const Gtk::BuilderError & err)
{
  RCLCPP_ERROR(rclcpp::get_logger(""), "Exception (Gtk::BuilderError) caught: %s\nTerminating ...", err.what().c_str());
  return EXIT_FAILURE;
}
catch (...)
{
  RCLCPP_ERROR(rclcpp::get_logger(""), "Unhandled exception caught.\nTerminating ...");
  return EXIT_FAILURE;
}

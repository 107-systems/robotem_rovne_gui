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

int main(int argc, char * argv[])
{
  glutInit(&argc, argv);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<t07::Node>();

  try
  {
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }
  catch (std::runtime_error const & err)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Exception (std::runtime_error) caught: %s\nTerminating ...", err.what());
    return EXIT_FAILURE;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Unhandled exception caught.\nTerminating ...");
    return EXIT_FAILURE;
  }
}
